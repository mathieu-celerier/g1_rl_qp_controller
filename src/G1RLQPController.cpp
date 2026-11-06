#include "G1RLQPController.h"

#include <RBDyn/MultiBodyConfig.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <fcntl.h>
#include <mc_joystick_plugin/joystick_inputs.h>
#include <termios.h>

#include <algorithm>
#include <cmath>

G1RLQPController::G1RLQPController(mc_rbdyn::RobotModulePtr rm,
                                     double dt,
                                     const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;

  // Optional override of the joint-limit CBF and velocity-limit parameters (defaults in the header)
  if(config_.has("joint_limit_cbf"))
  {
    const auto cbf = config_("joint_limit_cbf");
    cbf("zeta", zeta_jointLimit_);
    cbf("lambda", lambda_jointLimit_);
    cbf("ds_percent", dsPercent_);
    cbf("vel_percent", velPercent_);
    diPercent_ = std::max(diPercent_, std::min(dsPercent_ * 2.0, 0.999)); // di > ds
    mc_rtc::log::info("[G1RLQPController] joint-limit CBF: zeta={}, lambda={}, ds={}, velocity fraction={}",
                      zeta_jointLimit_, lambda_jointLimit_, dsPercent_, velPercent_);
  }

  //Initialize Constraints
  // These defaults provide joint-limit and self-collision protection
  updateConstraints();

  // The default posture task created by the FSM carries qdd* for acceleration policies (see
  // configureTasksForPolicy), it is out of the solver otherwise
  accelerationTask = getPostureTask(robot().name());
  if(!accelerationTask)
    mc_rtc::log::error_and_throw("[G1RLQPController] No FSM PostureTask for robot {}", robot().name());
  solver().removeTask(accelerationTask);
  // Initialize Task
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(
      solver(), robot().robotIndex(), 100.0, 1); // TODO(robot): tune stiffness/weight
  initializeRobotBasics();

  rlRuntime_.setPostureTask(accelerationTask);
  rlRuntime_.configure(config_, *this, torqueJointTask);
  configureTasksForPolicy();
  loadSimTest();

  addGui();
  addLog();

  mc_rtc::log::success("[G1RLQPController] init done");
}

bool G1RLQPController::run()
{
  // Use joystick plugin if present else jeyboard inputs
  controllerAvailable = datastore().has("Joystick::connected") && datastore().get<bool>("Joystick::connected");
  if(controllerAvailable && toggleJoystick)
    RLuseJoyStickInputs();
  else if(toggleKeyboard)
    RLuseKeyboardInputs();

  if(printLimits_) computeLimits();
  updateSimTest();
  bool run = mc_control::fsm::Controller::run(
          mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  if(byPassQPControl()) // Run RL without taking the QP into account
  {
    return true;
  }
  return run; // Return false if QP fails
}

void G1RLQPController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  rlRuntime_.reset(*this);
}

void G1RLQPController::RLuseJoyStickInputs()
{
  // Get joystick functions
  auto & stickFunc = datastore().get<std::function<Eigen::Vector2d(joystickAnalogicInputs)>>("Joystick::Stick");

  // Read sticks values
  leftStick = stickFunc(joystickAnalogicInputs::L_STICK);
  // Apply dead zone
  double vel_x = 0.0;
  if(std::abs(leftStick(0) - 0.5) > joystickDeadZone)
  {
    vel_x = (leftStick(0) - 0.5) * 2.0 * maxVelCmd;
  }
  double vel_y = 0.0;
  if(std::abs(leftStick(1) - 0.5) > joystickDeadZone)
  {
    vel_y = (leftStick(1) - 0.5) * 2.0 * maxVelCmd;
  }

  rightStick = stickFunc(joystickAnalogicInputs::R_STICK);
  double yaw_cmd = 0.0;
  if(std::abs(rightStick(1) - 0.5) > joystickDeadZone)
  {
    yaw_cmd = (rightStick(1) - 0.5) * 2.0 * maxYawCmd;
  }

  // Read D-pad buttons
  DirectionButtons = {datastore().get<bool>("Joystick::UpPad"), datastore().get<bool>("Joystick::DownPad"),
                      datastore().get<bool>("Joystick::LeftPad"), datastore().get<bool>("Joystick::RightPad")};

  for(size_t i = 0; i < DirectionButtons.size(); ++i)
  {
    if(DirectionButtons[i])
    {
      switch(i)
      {
        case 0: // Up
          vel_x += 1.0 * maxVelCmd;
          break;
        case 1: // Down
          vel_x -= 1.0 * maxVelCmd;
          break;
        case 2: // Left
          vel_y += 1.0 * maxVelCmd;
          break;
        case 3: // Right
          vel_y -= 1.0 * maxVelCmd;
          break;
        default:
          break;
      }
    }
  }
  rlRuntime_.setCommand({vel_x, vel_y, yaw_cmd});
}

void G1RLQPController::RLuseKeyboardInputs()
{
  struct Ctx
  {
    bool ready = false;
    termios old{};
    bool seen[4] = {};
    std::chrono::steady_clock::time_point ts[4];
    std::array<char, 64> buf{};
    size_t sz = 0;
  };
  static Ctx k;

  if(!k.ready)
  {
    if(::isatty(STDIN_FILENO) != 1) return;
    k.ready = true;
    ::tcgetattr(STDIN_FILENO, &k.old);
    termios raw = k.old;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = raw.c_cc[VTIME] = 0;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    ::fcntl(STDIN_FILENO, F_SETFL, ::fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
  }

  char tmp[32];
  ssize_t n = ::read(STDIN_FILENO, tmp, sizeof(tmp));
  if(n > 0 && k.sz + n < 64) std::copy(tmp, tmp + n, k.buf.begin() + k.sz), k.sz += n;

  auto now = std::chrono::steady_clock::now();
  for(size_t i = 0; i + 2 < k.sz; ++i)
    if(k.buf[i] == 27 && k.buf[i + 1] == '[')
    {
      int idx = k.buf[i + 2] == 'A'   ? 0
                : k.buf[i + 2] == 'B' ? 1
                : k.buf[i + 2] == 'D' ? 2
                : k.buf[i + 2] == 'C' ? 3
                                      : -1;
      if(idx >= 0) k.seen[idx] = true, k.ts[idx] = now;
      i += 2;
    }
  std::copy(k.buf.begin() + (k.sz > 2 ? k.sz - 2 : 0), k.buf.begin() + k.sz, k.buf.begin());
  k.sz = k.sz > 2 ? 2 : 0;

  const auto active = [&](int i)
  { return k.seen[i] && std::chrono::duration_cast<std::chrono::milliseconds>(now - k.ts[i]).count() < 500; };
  rlRuntime_.setCommand({(active(0) ? maxVelCmd : 0.0) - (active(1) ? maxVelCmd : 0.0),
                         (active(2) ? maxVelCmd : 0.0) - (active(3) ? maxVelCmd : 0.0), rlRuntime_.command()(2)});
}

void G1RLQPController::activateQPControl(bool activate)
{
  rlRuntime_.setUseQP(activate);
}

void G1RLQPController::loadPolicy(const std::string & policyName)
{
  rlRuntime_.loadPolicyByName(policyName, *this, torqueJointTask);
  configureTasksForPolicy();
}

void G1RLQPController::configureTasksForPolicy()
{
  if(rlRuntime_.accelerationAction())
  {
    if(torqueTaskActive_)
    {
      solver().removeTask(torqueJointTask);
      torqueTaskActive_ = false;
    }
    // No position or velocity feedback: the task only tracks refAccel = qdd*. The weight must be non-zero.
    accelerationTask->setGains(0.0, 0.0);
    accelerationTask->weight(rlRuntime_.postureWeight());
    // TVM's PostureFunction acts on the actuated joints only (refJointOrder), not on the floating base
    accelerationTask->reset();
    accelerationTask->refVel(Eigen::VectorXd::Zero(nbActuatedJoints));
    accelerationTask->refAccel(rlRuntime_.qdd_rl());
    if(!accelerationTaskActive_)
    {
      solver().addTask(accelerationTask);
      accelerationTaskActive_ = true;
    }
    mc_rtc::log::info("[G1RLQPController] Acceleration policy: PostureTask weight {}, |qdd*| <= {}",
                      accelerationTask->weight(), rlRuntime_.accelerationLimit());
  }
  else
  {
    if(accelerationTaskActive_)
    {
      solver().removeTask(accelerationTask);
      accelerationTaskActive_ = false;
    }
    if(!torqueTaskActive_)
    {
      solver().addTask(torqueJointTask);
      torqueTaskActive_ = true;
    }
  }
}

void G1RLQPController::loadSimTest()
{
  if(!config_.has("sim_test")) return;
  const auto st = config_("sim_test");
  simTestEnabled_ = true;
  if(st.has("command_schedule"))
  {
    for(const std::vector<double> & row : st("command_schedule"))
    {
      if(row.size() != 4)
        mc_rtc::log::error_and_throw("[G1RLQPController] sim_test.command_schedule rows are [t, vx, vy, wz]");
      commandSchedule_.push_back({{row[0], row[1], row[2], row[3]}});
    }
  }
  if(st.has("pushes"))
  {
    const auto p = st("pushes");
    pushesEnabled_ = true;
    p("body", pushBody_);
    p("at_com", pushAtCom_);
    p("force", pushForce_);
    p("duration", pushDuration_);
    p("interval", pushInterval_);
    pushRng_.seed(static_cast<unsigned>(p("seed", 0)));
    nextPushTime_ = p("start", 2.0);
  }
  mc_rtc::log::warning("[G1RLQPController] sim_test: {} command entries, pushes {} (body {}, {}-{} N)",
                       commandSchedule_.size(), pushesEnabled_, pushBody_, pushForce_[0], pushForce_[1]);
}

void G1RLQPController::updateSimTest()
{
  if(!simTestEnabled_) return;
  const double t = simTestTime_;
  simTestTime_ += timeStep;

  for(const auto & row : commandSchedule_)
  {
    if(row[0] <= t) rlRuntime_.setCommand({row[1], row[2], row[3]});
  }

  if(!pushesEnabled_) return;
  auto uniform = [this](const std::array<double, 2> & r)
  { return std::uniform_real_distribution<double>(r[0], r[1])(pushRng_); };
  if(t >= nextPushTime_)
  {
    const double angle = std::uniform_real_distribution<double>(-M_PI, M_PI)(pushRng_);
    const double force = uniform(pushForce_);
    pushForceVec_ << force * std::cos(angle), force * std::sin(angle), 0.0;
    pushEndTime_ = t + uniform(pushDuration_);
    nextPushTime_ = pushEndTime_ + uniform(pushInterval_);
  }
  if(t < pushEndTime_)
  {
    const auto call = robot().name() + "::ApplyForcesOnBody";
    Eigen::Vector3d pushPoint = Eigen::Vector3d::Zero(); // in the body frame
    if(pushAtCom_)
    {
      const auto & I = robot().mb().body(robot().bodyIndexByName(pushBody_)).inertia();
      pushPoint = I.momentum() / I.mass();
    }
    if(datastore().has(call))
    {
      datastore().call<bool, const std::string &, const sva::ForceVecd &, const Eigen::Vector3d>(
          call, pushBody_, sva::ForceVecd(Eigen::Vector3d::Zero(), pushForceVec_), pushPoint);
    }
  }
  else
    pushForceVec_.setZero();
}

rlqp::RLPolicyRuntime & G1RLQPController::rlRuntime() { return rlRuntime_; }

const rlqp::RLPolicyRuntime & G1RLQPController::rlRuntime() const { return rlRuntime_; }

void G1RLQPController::initializeRobotBasics()
{
  mc_rtc::log::info("[G1RLQPController] Using torque control mode");
  if(!datastore().has("ControlMode"))
  {
    datastore().make<std::string>("ControlMode", "Torque");
  }
  else
  {
    datastore().assign<std::string>("ControlMode", "Torque");
  }

  robotName_ = robot().name();
  jointNames = robot().refJointOrder();
  nbActuatedJoints = static_cast<int>(jointNames.size());
  if(!datastore().has("anchorFrameFunction"))
  {
    datastore().make_call("anchorFrameFunction", [this](const mc_rbdyn::Robot & real_robot) {return createContactAnchor(real_robot);});
  }
}

bool G1RLQPController::byPassQPControl()
{
  if(rlRuntime_.useQP()) return false; // QP is not bypassed, do nothing

  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  Eigen::VectorXd tau_rl = Eigen::VectorXd::Zero(nbActuatedJoints);
  const std::vector<std::vector<double> > & q_mbc = robot().mbc().q;
  const std::vector<std::vector<double> > & q_dot_mbc = robot().mbc().alpha;

  int i = 0;
  for(const auto &joint_name : jointNames)
  {
    const double q = q_mbc[robot().jointIndexByName(joint_name)][0];
    const double q_dot = q_dot_mbc[robot().jointIndexByName(joint_name)][0];
    tau_rl(i) = rlRuntime_.kp()(i) * (rlRuntime_.q_rl()(i) - q) - rlRuntime_.kd()(i) * q_dot;
    robot().mbc().jointTorque[robot().jointIndexByName(joint_name)][0] = tau_rl(i);
    i++;
  }

  return true;
}

void G1RLQPController::addLog()
{
  // Robot State variables
  logger().addLogEntry("G1RLQPController_kp_base", [this]() { return rlRuntime_.kpBase(); });
  logger().addLogEntry("G1RLQPController_kd_base", [this]() { return rlRuntime_.kdBase(); });
  logger().addLogEntry("G1RLQPController_kp_current", [this]() { return rlRuntime_.kp(); });
  logger().addLogEntry("G1RLQPController_kd_current", [this]() { return rlRuntime_.kd(); });
  logger().addLogEntry("G1RLQPController_pd_gains_ratio", [this]() { return rlRuntime_.pdGainsRatio(); });

  // RL variables
  logger().addLogEntry("G1RLQPController_RL_q", [this]() { return rlRuntime_.q_rl(); });
  logger().addLogEntry("G1RLQPController_RL_qZero", [this]() { return rlRuntime_.q_zero(); });
  logger().addLogEntry("G1RLQPController_RL_currentObservation", [this]() { return rlRuntime_.currentObservation(); });
  logger().addLogEntry("G1RLQPController_RL_currentAction", [this]() { return rlRuntime_.currentAction(); });
  logger().addLogEntry("G1RLQPController_RL_currentActionScaled", [this]() { return rlRuntime_.currentActionScaled(); });
  logger().addLogEntry("G1RLQPController_RL_actionScale", [this]() { return rlRuntime_.actionScale(); });
  logger().addLogEntry("G1RLQPController_RL_command", [this]() { return rlRuntime_.command(); });
  logger().addLogEntry("G1RLQPController_RL_qdd", [this]() { return rlRuntime_.qdd_rl(); });
  // Entry names of the previous G1RLQPController, read by the mc_mujoco test scripts
  logger().addLogEntry("G1RLQPController_velCmd", [this]() { return rlRuntime_.command(); });
  logger().addLogEntry("G1RLQPController_pushForce", [this]() { return pushForceVec_; });
  logger().addLogEntry("G1RLQPController_qddRefFull",
                       [this]()
                       {
                         // qdd* in mc_rtc dof order (floating base first), to compare with the QP output
                         Eigen::VectorXd out = Eigen::VectorXd::Zero(robot().mb().nrDof());
                         for(int i = 0; i < nbActuatedJoints; ++i)
                         {
                           const auto j = robot().jointIndexByName(jointNames[static_cast<size_t>(i)]);
                           out(robot().mb().jointPosInDof(static_cast<int>(j))) = rlRuntime_.qdd_rl()(i);
                         }
                         return out;
                       });

  // Controller state variables
  logger().addLogEntry("G1RLQPController_useQP", [this]() { return rlRuntime_.useQP(); });

  // Log current policy (name and convention)
  logger().addLogEntry("G1RLQPController_currentPolicy", [this]() { return rlRuntime_.currentPolicyName(); });
  logger().addLogEntry("G1RLQPController_observationConvention", [this]() { return rlRuntime_.conventionName(); });
  logger().addLogEntry("G1RLQPController_RL_phase", [this]() { return rlRuntime_.phase(); });
  logger().addLogEntry("G1RLQPController_RL_policy_period_s", [this]() { return rlRuntime_.policyStepSize(); });
  logger().addLogEntry("G1RLQPController_RL_update_count", [this]() { return rlRuntime_.policyUpdateCount(); });

  rlRuntime_.addLogObs(*this);
}

void G1RLQPController::addGui()
{
  gui()->addElement(
    {"G1RLQPController", "Policy"},
    mc_rtc::gui::Label("Current policy", [this]() { return rlRuntime_.currentPolicyName(); }),
    mc_rtc::gui::Label("Current policy folder", [this]() { return rlRuntime_.currentPolicyFolder(); }),
    mc_rtc::gui::Label("Observation convention", [this]() { return rlRuntime_.conventionName(); }),
    mc_rtc::gui::Label("Observation source", [this]() { return rlRuntime_.observationSource(); }),
    mc_rtc::gui::Label("Base body", [this]() { return rlRuntime_.baseBody(); }),
    mc_rtc::gui::Label("Observation size", [this]() { return rlRuntime_.observationSize(); }),
    mc_rtc::gui::Label("Action size", [this]() { return rlRuntime_.actionSize(); }),
    mc_rtc::gui::Label("Controlled action size", [this]() { return rlRuntime_.controlledActionSize(); }),
    mc_rtc::gui::ComboInput(
      "Select policy",
      rlRuntime_.availablePolicyNames(),
      [this]() { return rlRuntime_.currentPolicyName(); },
      [this](const std::string & policyName) { loadPolicy(policyName); }),
    mc_rtc::gui::Label("Action type",
                       [this]() { return rlRuntime_.accelerationAction() ? "acceleration" : "position"; }),
    mc_rtc::gui::Button("Reload current policy", [this]() {
      rlRuntime_.reloadCurrentPolicy(*this, torqueJointTask);
      configureTasksForPolicy();
    }));

  gui()->addElement(
    {"G1RLQPController", "PD Gains"},
    mc_rtc::gui::NumberSlider(
      "PD Gains Ratio",
      [this]() { return rlRuntime_.pdGainsRatio(); },
      [this](double v) { rlRuntime_.setPDGainsRatio(v, torqueJointTask); },
      0.0,
      2.0),
    mc_rtc::gui::Label("Current kp", [this]() { return rlRuntime_.kp(); }),
    mc_rtc::gui::Label("Current kd", [this]() { return rlRuntime_.kd(); }));

  gui()->addElement(
    {"G1RLQPController", "Control"},
    mc_rtc::gui::Button("Toggle QP Control", [this]() {
      rlRuntime_.setUseQP(!rlRuntime_.useQP());
    }),
    mc_rtc::gui::Label("QP Control", [this]() {
      return rlRuntime_.useQP() ? "Enforced" : "Bypassed";
    }),
    mc_rtc::gui::Button("Toggle print joint limits", [this]() {
      printLimits_ = !printLimits_;
    }),
    mc_rtc::gui::Label("Print joint limits", [this]() {
      return printLimits_ ? "Enabled" : "Disabled";
    }),
    mc_rtc::gui::NumberInput(
      "Zeta self-collision",
      [this]() { return zeta_selfCollision_; },
      [this](double v) { 
        zeta_selfCollision_ = v;
        updateSelfCollisionConstraints();
      }),
    mc_rtc::gui::NumberInput(
      "Lambda self-collision",
      [this]() { return lambda_selfCollision_; },
      [this](double v) { 
        lambda_selfCollision_ = v;
        updateSelfCollisionConstraints();
      }),
    mc_rtc::gui::NumberInput(
      "Zeta joint limits",
      [this]() { return zeta_jointLimit_; },
      [this](double v) { 
        zeta_jointLimit_ = v;
        updateDynamicConstraints();
      }),
    mc_rtc::gui::NumberInput(
      "Lambda joint limits",
      [this]() { return lambda_jointLimit_; },
      [this](double v) {
        lambda_jointLimit_ = v;
        updateDynamicConstraints();
      }),
    mc_rtc::gui::NumberInput(
      "Joint velocity limit (fraction of max)",
      [this]() { return velPercent_; },
      [this](double v) {
        velPercent_ = std::min(v, 0.999);
        mc_rtc::log::info("[G1RLQPController] Joint velocity limit fraction set to {}", velPercent_);
        updateDynamicConstraints();
      }),
    mc_rtc::gui::NumberInput(
      "Joint limit stop distance (fraction of range)",
      [this]() { return dsPercent_; },
      [this](double v) {
        dsPercent_ = std::min(v, 0.99);
        diPercent_ = std::min(dsPercent_ * 2.0, 0.999); // Ensure di > ds 
        mc_rtc::log::info("[G1RLQPController] ds set to {}", dsPercent_);
        mc_rtc::log::info("[G1RLQPController] di set to {}", diPercent_);
        updateDynamicConstraints();
      })
  );

  gui()->addElement(
    {"G1RLQPController", "Runtime"},
    mc_rtc::gui::Label("Controller period [s]", [this]() { return timeStep; }),
    mc_rtc::gui::Label("Controller rate [Hz]", [this]() { return 1.0 / timeStep; }),
    mc_rtc::gui::Label("Policy period [s]", [this]() { return rlRuntime_.policyStepSize(); }),
    mc_rtc::gui::Label("Policy rate [Hz]", [this]() { return rlRuntime_.policyRate(); }),
    mc_rtc::gui::Label("Policy updates", [this]() { return rlRuntime_.policyUpdateCount(); }),
    mc_rtc::gui::Label("Phase", [this]() { return rlRuntime_.phase(); }));

  gui()->addElement(
    {"G1RLQPController", "Command"},
    mc_rtc::gui::Button(
        "Toggle Joystick Plugin",
        [this]()
        {
          toggleJoystick = !toggleJoystick;
          if(toggleJoystick)
            toggleKeyboard = false;
        }),
    mc_rtc::gui::Label(
        "Current velocity control mode",
        [this]()
        {
          if(toggleKeyboard)
            return std::string{"Keyboard"};
          if(toggleJoystick && controllerAvailable)
            return std::string{"mc_joystick_plugin"};
          return std::string{"GUI"};
        }),
    mc_rtc::gui::Button(
        "Toggle Keyboard",
        [this]()
        {
          toggleKeyboard = !toggleKeyboard;
          if(toggleKeyboard)
            toggleJoystick = false;
        }),
    mc_rtc::gui::Label(
        "Joystick plugin available",
        [this]() { return controllerAvailable ? "Yes" : "No"; }),
    mc_rtc::gui::Label("Command values", []() { return std::string(" "); }),
    mc_rtc::gui::NumberInput(
      "vx",
      [this]() { return rlRuntime_.command()(0); },
      [this](double v) { rlRuntime_.command()(0) = v; }),
    mc_rtc::gui::NumberInput(
      "vy",
      [this]() { return rlRuntime_.command()(1); },
      [this](double v) { rlRuntime_.command()(1) = v; }),
    mc_rtc::gui::NumberInput(
      "yaw_rate",
      [this]() { return rlRuntime_.command()(2); },
      [this](double v) { rlRuntime_.command()(2) = v; }),
    mc_rtc::gui::Label("Max values", []() { return std::string(" "); }),
    mc_rtc::gui::NumberInput(
        "max_vel_cmd",
        [this]() { return maxVelCmd; }, [this](double v) { maxVelCmd = v; }),
    mc_rtc::gui::NumberInput(
        "max_yaw_cmd",
        [this]() { return maxYawCmd; }, [this](double v) { maxYawCmd = v; }));
}

void G1RLQPController::computeLimits()
{
  const double epsilon = 1e-5;

  mc_rbdyn::Robot & real_robot = realRobot(robotName_);

  const std::vector<std::vector<double> > & currentPos = real_robot.q();
  const std::vector<std::vector<double> > & currentVel = real_robot.alpha();
  const std::vector<std::vector<double> > & currentTau = real_robot.jointTorque();

  const std::vector<std::vector<double> > & qLimLower = real_robot.ql();
  const std::vector<std::vector<double> > & qLimUpper = real_robot.qu();

  const std::vector<std::vector<double> > & qDotLimLower = real_robot.vl();
  const std::vector<std::vector<double> > & qDotLimUpper = real_robot.vu();

  const std::vector<std::vector<double> > & tauLimLower = real_robot.tl();
  const std::vector<std::vector<double> > & tauLimUpper = real_robot.tu();

  for(size_t j = 0; j < jointNames.size(); ++j)
  {
    const std::string & joint = jointNames[j];
    const int i = real_robot.jointIndexByName(joint);
    const size_t idx = static_cast<size_t>(i);

    const double ds = dsPercent_ * (qLimUpper[idx][0] - qLimLower[idx][0]);

    const double posLimitUp = qLimUpper[idx][0] - ds;
    const double posLimitLow = qLimLower[idx][0] + ds;

    const double velLimitUp = velPercent_ * qDotLimUpper[idx][0];
    const double velLimitLow = velPercent_ * qDotLimLower[idx][0];

    const double tauLimitUp = tauLimUpper[idx][0];
    const double tauLimitLow = tauLimLower[idx][0];

    if(currentPos[idx][0] > posLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[G1RLQPController] Joint {} position upper limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitUp);
    }

    if(currentPos[idx][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[G1RLQPController] Joint {} position lower limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitLow);
    }

    if(currentVel[idx][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[G1RLQPController] Joint {} velocity upper limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitUp);
    }

    if(currentVel[idx][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[G1RLQPController] Joint {} velocity lower limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitLow);
    }

    if(currentTau[idx][0] > tauLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[G1RLQPController] Joint {} torque upper limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitUp);
    }

    if(currentTau[idx][0] < tauLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[G1RLQPController] Joint {} torque lower limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitLow);
    }
  }
}

std::pair<sva::PTransformd, Eigen::Vector3d> G1RLQPController::createContactAnchor(const mc_rbdyn::Robot & anchorRobot)
{
  // Anchor between the two sole centers, weighted by the knee external torque of the estimated (real) robot, as
  // validated with the acceleration policies in mc_mujoco (Tilt base velocity error 0.011-0.014 m/s)
  const auto & real = realRobot(robotName_);
  const Eigen::VectorXd & tauExt = real.externalTorques();
  auto kneeTorque = [&](const char * joint)
  {
    if(tauExt.size() != real.mb().nrDof()) return 1.0;
    return std::abs(tauExt(real.mb().jointPosInDof(static_cast<int>(real.jointIndexByName(joint)))));
  };
  double wl = kneeTorque("left_knee_joint");
  double wr = kneeTorque("right_knee_joint");
  if(wl + wr < 1e-6) wl = wr = 1.0;
  const double leftFootRatio = wl / (wl + wr);

  // Position and world velocity of a sole center (velocity of that point, not of the ankle body origin)
  auto solePoint = [&](const char * surface, Eigen::Vector3d & p, Eigen::Vector3d & v)
  {
    const auto & body = anchorRobot.surface(surface).bodyName();
    const auto & velW = anchorRobot.bodyVelW(body);
    p = anchorRobot.surfacePose(surface).translation();
    v = velW.linear() + velW.angular().cross(p - anchorRobot.bodyPosW(body).translation());
  };
  Eigen::Vector3d p_l, v_l, p_r, v_r;
  solePoint("LeftFootCenter", p_l, v_l);
  solePoint("RightFootCenter", p_r, v_r);

  contactAnchorTf_ =
      sva::PTransformd(Eigen::Matrix3d::Identity(), leftFootRatio * p_l + (1 - leftFootRatio) * p_r);
  return {contactAnchorTf_, leftFootRatio * v_l + (1 - leftFootRatio) * v_r};
}

void G1RLQPController::updateDynamicConstraints()
{
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);
}

void G1RLQPController::updateSelfCollisionConstraints()
{
  selfCollisionConstraint->setCollisionsDampers(solver(), {zeta_selfCollision_, lambda_selfCollision_});
}

void G1RLQPController::updateConstraints()
{
  updateSelfCollisionConstraints();
  updateDynamicConstraints();
}
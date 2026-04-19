#include "studica_ros2_control/diff_drive_component.h"

namespace studica_ros2_control {

std::shared_ptr<rclcpp::Node> DiffDrive::initialize(rclcpp::Node *control, std::shared_ptr<DiffOdometry> odom, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("diff_drive_component.name", "");
    control->declare_parameter<int>("diff_drive_component.can_id", -1);
    control->declare_parameter<int>("diff_drive_component.motor_freq", -1);
    control->declare_parameter<int>("diff_drive_component.ticks_per_rotation", -1);
    control->declare_parameter<float>("diff_drive_component.wheel_radius", -1.0);
    control->declare_parameter<float>("diff_drive_component.wheelbase", -1.0);
    control->declare_parameter<float>("diff_drive_component.width", -1.0);
    control->declare_parameter<int>("diff_drive_component.front_left", -1);
    control->declare_parameter<int>("diff_drive_component.front_right", -1);
    control->declare_parameter<int>("diff_drive_component.rear_left", -1);
    control->declare_parameter<int>("diff_drive_component.rear_right", -1);
    control->declare_parameter<bool>("diff_drive_component.invert_front_left", false);
    control->declare_parameter<bool>("diff_drive_component.invert_front_right", false);
    control->declare_parameter<bool>("diff_drive_component.invert_rear_left", false);
    control->declare_parameter<bool>("diff_drive_component.invert_rear_right", false);
    control->declare_parameter<std::string>("diff_drive_component.titan_control_mode", "open_loop");

    std::string name = control->get_parameter("diff_drive_component.name").as_string();
    int can_id = control->get_parameter("diff_drive_component.can_id").as_int();
    int motor_freq = control->get_parameter("diff_drive_component.motor_freq").as_int();
    int ticks_per_rotation = control->get_parameter("diff_drive_component.ticks_per_rotation").as_int();
    float wheel_radius = control->get_parameter("diff_drive_component.wheel_radius").get_value<float>();
    float wheelbase = control->get_parameter("diff_drive_component.wheelbase").get_value<float>();
    float width = control->get_parameter("diff_drive_component.width").get_value<float>();
    int fl = control->get_parameter("diff_drive_component.front_left").as_int();
    int fr = control->get_parameter("diff_drive_component.front_right").as_int();
    int rl = control->get_parameter("diff_drive_component.rear_left").as_int();
    int rr = control->get_parameter("diff_drive_component.rear_right").as_int();
    bool invert_fl = control->get_parameter("diff_drive_component.invert_front_left").as_bool();
    bool invert_fr = control->get_parameter("diff_drive_component.invert_front_right").as_bool();
    bool invert_rl = control->get_parameter("diff_drive_component.invert_rear_left").as_bool();
    bool invert_rr = control->get_parameter("diff_drive_component.invert_rear_right").as_bool();
    std::string titan_control_mode = control->get_parameter("diff_drive_component.titan_control_mode").as_string();

    RCLCPP_INFO(control->get_logger(), "%s -> fl: %d, fr: %d, rl: %d, rr: %d", name.c_str(), fl, fr, rl, rr);

    auto diff_drive_node = std::make_shared<DiffDrive>(vmx, odom, name, can_id, motor_freq, ticks_per_rotation, wheel_radius, wheelbase, width, fl, fr, rl, rr, invert_fl, invert_fr, invert_rl, invert_rr, titan_control_mode);
    return diff_drive_node;
}

DiffDrive::DiffDrive(const rclcpp::NodeOptions & options) : Node("diff_drive", options) {}

DiffDrive::DiffDrive(
    std::shared_ptr<VMXPi> vmx,
    std::shared_ptr<studica_ros2_control::DiffOdometry> odom,
    const std::string &name,
    const uint8_t &can_id,
    const uint16_t &motor_freq,
    const uint16_t &ticks_per_rotation,
    const float &wheel_radius,
    const float &wheelbase,
    const float &width,
    const uint8_t &front_left,
    const uint8_t &front_right,
    const uint8_t &rear_left,
    const uint8_t &rear_right,
    const bool &invert_front_left,
    const bool &invert_front_right,
    const bool &invert_rear_left,
    const bool &invert_rear_right,
    const std::string &titan_control_mode) 
    : Node(name),
      vmx_(vmx),
      odom_(odom),
      can_id_(can_id),
      motor_freq_(motor_freq),
      ticks_per_rotation_(ticks_per_rotation),
      wheel_radius_(wheel_radius),
      wheelbase_(wheelbase),
      width_(width),
      fl_(front_left),
      fr_(front_right),
      rl_(rear_left),
      rr_(rear_right),
      titan_control_mode_(TitanControlMode::OPEN_LOOP),
      titan_control_mode_name_("open_loop"),
      have_last_cmd_time_(false),
      last_cmd_time_(this->now()),
      fl_target_counts_(0.0),
      fr_target_counts_(0.0),
      rl_target_counts_(0.0),
      rr_target_counts_(0.0) {

    // Arc length per encoder tick — used for odometry and position-mode target integration.
    dist_per_tick_ = 2 * M_PI * wheel_radius_ / ticks_per_rotation_;

    titan_ = std::make_shared<studica_driver::Titan>(can_id_, motor_freq_, dist_per_tick_, vmx_);

    service_ = this->create_service<studica_ros2_control::srv::SetData>(
        "titan_cmd",
        std::bind(&DiffDrive::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    titan_->ConfigureEncoder(fl_, dist_per_tick_);
    titan_->ConfigureEncoder(fr_, dist_per_tick_);
    titan_->ConfigureEncoder(rl_, dist_per_tick_);
    titan_->ConfigureEncoder(rr_, dist_per_tick_);

    titan_->ResetEncoder(fl_);
    titan_->ResetEncoder(fr_);
    titan_->ResetEncoder(rl_);
    titan_->ResetEncoder(rr_);

    if (invert_front_left) titan_->InvertMotor(fl_);
    if (invert_front_right) titan_->InvertMotor(fr_);
    if (invert_rear_left) titan_->InvertMotor(rl_);
    if (invert_rear_right) titan_->InvertMotor(rr_);

    titan_->Enable(true);
    // Apply PID type and position hold to device before accepting /cmd_vel commands.
    set_titan_control_mode(titan_control_mode);

    odom_->setWheelParams(wheelbase_);
    odom_->init(this->now());

    // Odometry published at 20 Hz; also satisfies Titan 200 ms keep-alive when combined with cmd_vel.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DiffDrive::publish_odometry, this));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&DiffDrive::cmd_vel_callback, this, std::placeholders::_1)
    );
}

DiffDrive::~DiffDrive() {}

void DiffDrive::set_titan_control_mode(const std::string &mode_name) {
    const std::string requested_mode = mode_name;
    if (mode_name == "open_loop") {
        titan_control_mode_ = TitanControlMode::OPEN_LOOP;
        titan_control_mode_name_ = "open_loop";
    } else if (mode_name == "velocity") {
        titan_control_mode_ = TitanControlMode::VELOCITY;
        titan_control_mode_name_ = "velocity";
    } else if (mode_name == "position") {
        titan_control_mode_ = TitanControlMode::POSITION;
        titan_control_mode_name_ = "position";
    } else {
        titan_control_mode_ = TitanControlMode::OPEN_LOOP;
        titan_control_mode_name_ = "open_loop";
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid diff_drive_component.titan_control_mode='%s'. Falling back to 'open_loop'.",
            mode_name.c_str());
    }

    if (titan_control_mode_ == TitanControlMode::POSITION && !titan_->SupportsPIDType(2)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Requested mode '%s' needs PID type 2, but firmware does not support it. Falling back to 'velocity' (PID type 1).",
            requested_mode.c_str());
        titan_control_mode_ = TitanControlMode::VELOCITY;
        titan_control_mode_name_ = "velocity";
    }

    apply_titan_pid_mode();
    have_last_cmd_time_ = false;
    last_cmd_time_ = this->now();
    // Seed position targets from current encoder values to avoid an instant jump on mode switch.
    fl_target_counts_ = static_cast<double>(titan_->GetEncoderCount(fl_));
    fr_target_counts_ = static_cast<double>(titan_->GetEncoderCount(fr_));
    rl_target_counts_ = static_cast<double>(titan_->GetEncoderCount(rl_));
    rr_target_counts_ = static_cast<double>(titan_->GetEncoderCount(rr_));

    RCLCPP_INFO(
        this->get_logger(),
        "Diff drive Titan control mode: %s",
        titan_control_mode_name_.c_str());
}

void DiffDrive::apply_titan_pid_mode() {
    if (titan_control_mode_ == TitanControlMode::OPEN_LOOP) {
        titan_->SetPIDType(0);  // SetSpeed duty-cycle mode
        titan_->SetPositionHold(fl_, false);
        titan_->SetPositionHold(fr_, false);
        titan_->SetPositionHold(rl_, false);
        titan_->SetPositionHold(rr_, false);
        return;
    }

    // Closed-loop PID selection:
    // 1 -> velocity PID, 2 -> position PID.
    const bool hold_position = (titan_control_mode_ == TitanControlMode::POSITION);
    titan_->SetPIDType(hold_position ? 2 : 1);
    titan_->SetPositionHold(fl_, hold_position);
    titan_->SetPositionHold(fr_, hold_position);
    titan_->SetPositionHold(rl_, hold_position);
    titan_->SetPositionHold(rr_, hold_position);
}

int16_t DiffDrive::linear_mps_to_rpm(double linear_mps) const {
    if (wheel_radius_ <= 0.0f) {
        return 0;
    }
    const double wheel_rps = linear_mps / (2.0 * M_PI * static_cast<double>(wheel_radius_));
    double rpm = wheel_rps * 60.0;
    if (rpm > 32767.0) rpm = 32767.0;
    if (rpm < -32768.0) rpm = -32768.0;
    return static_cast<int16_t>(std::lround(rpm));
}

void DiffDrive::command_motor(uint8_t motor, double linear_mps, double dt_sec, double *target_counts) {
    switch (titan_control_mode_) {
        case TitanControlMode::OPEN_LOOP:
            // linear_mps is used directly as duty (-1..1 or m/s scaled by caller).
            titan_->SetSpeed(motor, linear_mps);
            return;
        case TitanControlMode::VELOCITY:
            titan_->SetTargetVelocity(motor, linear_mps_to_rpm(linear_mps));
            return;
        case TitanControlMode::POSITION:
            if (target_counts == nullptr || dist_per_tick_ <= 0.0f) {
                return;
            }
            // Integrate velocity × dt into encoder counts and send the running total.
            // Titan PID holds that absolute count; stopping cmd_vel naturally holds position.
            *target_counts += (linear_mps * dt_sec) / dist_per_tick_;
            titan_->SetTargetDistance(motor, static_cast<int32_t>(std::lround(*target_counts)));
            return;
    }
}

void DiffDrive::cmd_callback(std::shared_ptr<studica_ros2_control::srv::SetData::Request> request, std::shared_ptr<studica_ros2_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, request, response);
}

void DiffDrive::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    // For differential drive, wheelbase is the distance between left and right wheels
    double left_command = linear - angular * wheelbase_ / 2.0;
    double right_command = linear + angular * wheelbase_ / 2.0;

    double dt_sec = 0.05;
    const auto current_time = this->now();
    if (have_last_cmd_time_) {
        dt_sec = (current_time - last_cmd_time_).seconds();
    }
    last_cmd_time_ = current_time;
    have_last_cmd_time_ = true;
    // Clamp dt to a sane range: negative or huge values (e.g. first tick, clock jump) break position integration.
    if (dt_sec <= 0.0 || dt_sec > 0.5) {
        dt_sec = 0.05;
    }

    command_motor(fl_, left_command, dt_sec, &fl_target_counts_);
    command_motor(rl_, left_command, dt_sec, &rl_target_counts_);
    command_motor(fr_, right_command, dt_sec, &fr_target_counts_);
    command_motor(rr_, right_command, dt_sec, &rr_target_counts_);
}

void DiffDrive::cmd(std::string params, std::shared_ptr<studica_ros2_control::srv::SetData::Request> request, std::shared_ptr<studica_ros2_control::srv::SetData::Response> response) {
    if (params == "enable") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan enabled";
    } else if (params == "set_mode") {
        std::string mode_name = request->name;
        if (mode_name.empty()) {
            mode_name = request->command;
        }
        set_titan_control_mode(mode_name);
        response->success = true;
        response->message = "Titan mode set to " + titan_control_mode_name_;
    } else if (params == "set_target_velocity") {
        const int16_t rpm = static_cast<int16_t>(std::lround(request->initparams.speed));
        titan_->SetTargetVelocity(request->initparams.n_encoder, rpm);
        response->success = true;
        response->message = "Encoder " + std::to_string(request->initparams.n_encoder) +
            " target velocity set to " + std::to_string(static_cast<int>(rpm)) + " rpm";
    } else if (params == "disable") {
        titan_->Enable(false);
        response->success = true;
        response->message = "Titan disabled";
    } else if (params == "start") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan started";
    } else if (params == "setup_encoder") {
        titan_->SetupEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan encoder setup complete";
    } else if (params == "configure_encoder") {
        titan_->ConfigureEncoder(request->initparams.n_encoder, request->initparams.dist_per_tick);
        response->success = true;
        response->message = "Titan encoder configured";
    } else if (params == "stop") {
        if (titan_control_mode_ == TitanControlMode::OPEN_LOOP) {
            titan_->SetSpeed(request->initparams.n_encoder, 0.0);
        } else if (titan_control_mode_ == TitanControlMode::VELOCITY) {
            titan_->SetTargetVelocity(request->initparams.n_encoder, 0);
        } else {
            titan_->SetTargetDistance(
                request->initparams.n_encoder,
                titan_->GetEncoderCount(request->initparams.n_encoder));
        }
        response->success = true;
        response->message = "Titan stopped";
    } else if (params == "reset") {
        titan_->ResetEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan reset";
    } else if (params == "set_speed") {
        response->success = true;
        float speed = request->initparams.speed;
        if (titan_control_mode_ == TitanControlMode::OPEN_LOOP) {
            RCLCPP_INFO(this->get_logger(), "Setting open-loop speed to %f", speed);
            titan_->SetSpeed(request->initparams.n_encoder, speed);
            response->message = "Encoder " + std::to_string(request->initparams.n_encoder) +
                " open-loop speed set to " + std::to_string(request->initparams.speed);
        } else if (titan_control_mode_ == TitanControlMode::VELOCITY) {
            const int16_t rpm = linear_mps_to_rpm(speed);
            titan_->SetTargetVelocity(request->initparams.n_encoder, rpm);
            response->message = "Encoder " + std::to_string(request->initparams.n_encoder) +
                " target velocity set to " + std::to_string(static_cast<int>(rpm)) + " rpm";
        } else {
            const int32_t current_counts = titan_->GetEncoderCount(request->initparams.n_encoder);
            const int32_t target_counts = current_counts + static_cast<int32_t>(std::lround(speed));
            titan_->SetTargetDistance(request->initparams.n_encoder, target_counts);
            response->message = "Encoder " + std::to_string(request->initparams.n_encoder) +
                " target distance offset set by " + std::to_string(static_cast<int>(std::lround(speed))) + " ticks";
        }
    } else if (params == "get_encoder_distance") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderDistance(request->initparams.n_encoder));
    } else if (params == "get_rpm") {
        response->success = true;
        response->message = std::to_string(titan_->GetRPM(request->initparams.n_encoder));
    } else if (params == "get_encoder_count") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderCount(request->initparams.n_encoder));
    } else if (params == "autotune") {
        // Runs on-device PID autotune. Motors will move — run with robot lifted or in open space.
        titan_->AutotuneAll();
        response->success = true;
        response->message = "AutotuneAll sent — motors may move during tuning";
    } else if (params == "set_sensitivity") {
        const uint8_t sensitivity = static_cast<uint8_t>(
            std::clamp(static_cast<int>(std::lround(request->initparams.speed)), 0, 255));
        titan_->SetSensitivity(request->initparams.n_encoder, sensitivity);
        response->success = true;
        response->message = "Motor " + std::to_string(request->initparams.n_encoder) +
            " sensitivity set to " + std::to_string(static_cast<int>(sensitivity));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void DiffDrive::publish_odometry() {
    // Get encoder distances from all 4 motors
    double fl_encoder = titan_->GetEncoderDistance(fl_);
    double fr_encoder = titan_->GetEncoderDistance(fr_);
    double rl_encoder = titan_->GetEncoderDistance(rl_);
    double rr_encoder = titan_->GetEncoderDistance(rr_);

    // Average left and right sides for differential drive
    double left_encoder = (fl_encoder + rl_encoder) / 2.0;
    double right_encoder = (fr_encoder + rr_encoder) / 2.0;

    auto current_time = this->now();

    odom_->updateAndPublish(left_encoder, right_encoder, current_time);
}

} // namespace studica_ros2_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_ros2_control::DiffDrive)

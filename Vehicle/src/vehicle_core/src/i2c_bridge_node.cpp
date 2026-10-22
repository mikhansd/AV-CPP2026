#include "vehicle_core/i2c_bridge_node.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <chrono>

#include <vehicle_core/Devices/I2C/I2CBusRegistry.hpp>

using rclcpp::QoS;
using rclcpp::KeepLast;
using rclcpp::SensorDataQoS;

namespace vehicle_core {

rclcpp::QoS I2cBridgeNode::qos_transient_reliable(size_t depth) const {
  rclcpp::QoS q{rclcpp::KeepLast(depth)};
  q.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  q.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  return q;
}

I2cBridgeNode::I2cBridgeNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("i2c_bridge_node", opts)
{
  // Params
  i2c_path_              = this->declare_parameter<std::string>("i2c.path", "/dev/i2c-1");
  addr_lighting_         = static_cast<uint8_t>(this->declare_parameter<int>("i2c.addr_lighting", 0x15));
  addr_drive_            = static_cast<uint8_t>(this->declare_parameter<int>("i2c.addr_drive",    0x20));
  addr_imu_              = static_cast<uint8_t>(this->declare_parameter<int>("i2c.addr_imu",      0x29));
  drive_cmd_hz_          = this->declare_parameter<int>("rates.drive_cmd_hz", 20);
  drive_status_hz_       = this->declare_parameter<int>("rates.drive_status_hz", 20);
  imu_data_hz_           = this->declare_parameter<int>("rates.imu_data_hz", 50);
  brake_light_threshold_ = this->declare_parameter<int>("lighting.brake_threshold_percent", 10);
  default_enable_        = this->declare_parameter<bool>("drive.default_enable", true);

  // Open I2C
  i2cBus1 = I2CBusRegistry::getInstance().getOrOpen(i2c_path_);
  if (!i2cBus1) 
    throw std::runtime_error("Failed to open I2C bus");
  if (!drive.open(i2cBus1, addr_drive_))
    throw std::runtime_error("Failed to open drive");
  drive.startLogging("drive_log.csv");
  if (!lighting.open(i2cBus1, addr_lighting_)) 
    throw std::runtime_error("Failed to open lighting");
  if (!imu.open(i2cBus1, addr_imu_)) 
    throw std::runtime_error("Failed to open BNO055 IMU");

  // initialize IMU
  imu.init_ndof();

  // Publishers
  pub_drive_status_     = this->create_publisher<vehicle_core::msg::DriveStatus>("/vehicle/drive_status", 10);
  pub_drive_feedback_   = this->create_publisher<vehicle_core::msg::DriveFeedback>("/vehicle/drive_feedback", SensorDataQoS());
  pub_drive_fault_bits_ = this->create_publisher<std_msgs::msg::UInt16>("/vehicle/drive_fault_bits", qos_transient_reliable(1));
  pub_lights_status_    = this->create_publisher<vehicle_core::msg::LightsStatus>("/vehicle/lights_status", 1);
  pub_imu_data_         = this->create_publisher<sensor_msgs::msg::Imu>("/vehicle/imu/data", SensorDataQoS());
  pub_imu_mag_          = this->create_publisher<sensor_msgs::msg::MagneticField>("/vehicle/imu/mag", SensorDataQoS());
  pub_imu_status_       = this->create_publisher<vehicle_core::msg::IMUStatus>("/vehicle/imu/status", qos_transient_reliable(1));
  pub_velocity_         = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/velocity", SensorDataQoS());

  // Subscriptions
  sub_state_mode_ = this->create_subscription<vehicle_core::msg::StateMode>(
      "/vehicle/state_mode", 10, std::bind(&I2cBridgeNode::onStateMode, this, std::placeholders::_1));

  sub_drive_target_ = this->create_subscription<vehicle_core::msg::DriveTarget>(
      "/vehicle/drive_target", 20, std::bind(&I2cBridgeNode::on_drive_target, this, std::placeholders::_1));

  sub_lights_ = this->create_subscription<vehicle_core::msg::LightsCommand>(
      "/vehicle/lights_cmd", 5, std::bind(&I2cBridgeNode::on_lights, this, std::placeholders::_1));

  sub_tcp_state_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/tcp_server/connection_state", qos_transient_reliable(1),
      std::bind(&I2cBridgeNode::on_tcp_state, this, std::placeholders::_1));

  using namespace std::chrono;
  auto ms = [](int hz){ return std::chrono::milliseconds(std::max(1, 1000 / std::max(1, hz))); };

  // Timers
  t_drive_cmd_     = this->create_wall_timer(ms(drive_cmd_hz_),          std::bind(&I2cBridgeNode::tick_drive_cmd, this));
  t_drive_status_  = this->create_wall_timer(ms(drive_status_hz_),       std::bind(&I2cBridgeNode::tick_drive_status, this));
  t_imu_data       = this->create_wall_timer(ms(imu_data_hz_),           std::bind(&I2cBridgeNode::tick_imu_data_, this));

  RCLCPP_INFO(get_logger(), "i2c_bridge_node up on %s  drive=0x%02X lighting=0x%02X imu=0x%02X",
              i2c_path_.c_str(), addr_drive_, addr_lighting_, addr_imu_);

  {
    std::lock_guard<std::mutex> lks(lights_mtx_);
    publish_lights_status_locked();
  }

  // mag_csv_.open("mag_log.csv", std::ios::out | std::ios::trunc);
  // if (mag_csv_) {
  //   mag_csv_ << "t_sec,mx_T,my_T,mz_T,qw,qx,qy,qz,heading_deg_level\n";
  //   mag_csv_.flush();
  //   mag_csv_opened_ = true;
  // } else {
  //   RCLCPP_WARN(get_logger(), "Could not open mag_log.csv for writing");
  // }
}

// Callbacks
void I2cBridgeNode::onStateMode(const vehicle_core::msg::StateMode::SharedPtr m) {
  if (m->mode == 5 /*ESTOP*/) 
    estop_.store(true); 
  else
    estop_.store(false);
}

void I2cBridgeNode::on_drive_target(const vehicle_core::msg::DriveTarget::SharedPtr m) {
  last_target_ = m;
  maybe_auto_brake_lights(m->brake_percent);
}

void I2cBridgeNode::on_lights(const vehicle_core::msg::LightsCommand::SharedPtr m) {
  // Interpret incoming as LightID toggle intent
  const uint8_t id = m->light_id;
  last_light_cmd_ = id; // keep for visibility/diagnostics

  bool changed = false;
  uint8_t cmd_to_send = 0;

  {
    std::lock_guard<std::mutex> lks(lights_mtx_);

    switch (id) {
      case 0x01: { // LeftTurnSig toggle
        // If hazards are on, turning one side typically cancels hazards first.
        if (lights_.hazards) {
          lights_.hazards = false;
          // No need to send HazardsOff if your MCU derives hazards from both signals on.
          // If it needs explicit HazardsOff, uncomment next line:
          // send_light_cmd(static_cast<uint8_t>(LightingCommand::HazardsOff));
        }
        lights_.left_sig = !lights_.left_sig;
        cmd_to_send = static_cast<uint8_t>(
            lights_.left_sig ? 3 /*LeftSigOn*/ : 4 /*LeftSigOff*/);
        changed = true;
        break;
      }
      case 0x02: { // RightTurnSig toggle
        if (lights_.hazards) {
          lights_.hazards = false;
          // Optionally: send HazardsOff explicitly
          // send_light_cmd(static_cast<uint8_t>(LightingCommand::HazardsOff));
        }
        lights_.right_sig = !lights_.right_sig;
        cmd_to_send = static_cast<uint8_t>(
            lights_.right_sig ? 5 /*RightSigOn*/ : 6 /*RightSigOff*/);
        changed = true;
        break;
      }
      case 0x03: { // Headlights toggle
        lights_.headlights = !lights_.headlights;
        cmd_to_send = static_cast<uint8_t>(
            lights_.headlights ? 1 /*HeadlightsOn*/ : 2 /*HeadlightsOff*/);
        changed = true;
        break;
      }
      case 0x04: { // Hazards toggle (example ID)
        lights_.hazards = !lights_.hazards;
        // When hazards on, most cars cancel individual signals
        lights_.left_sig  = lights_.hazards;
        lights_.right_sig = lights_.hazards;
        cmd_to_send = static_cast<uint8_t>(
            lights_.hazards ? 7 /*HazardsOn*/ : 8 /*HazardsOff*/);
        changed = true;
        break;
      }
      case 0x00: // NoInput
      default:
        // ignore
        break;
    }

    if (changed) {
      // Send the single-byte command; if it fails, revert the state change so status stays truthful
      if (!send_light_cmd(cmd_to_send)) {
        // revert toggle if write failed
        switch (id) {
          case 0x01: lights_.left_sig = !lights_.left_sig; break;
          case 0x02: lights_.right_sig = !lights_.right_sig; break;
          case 0x03: lights_.headlights = !lights_.headlights; break;
          default: break;
        }
        RCLCPP_WARN(get_logger(), "Lighting I2C write failed; state reverted (errno=%d: %s)", errno, std::strerror(errno));
      } else {
        // on success, publish new status
        publish_lights_status_locked();
      }
    }
  }
}

void I2cBridgeNode::on_tcp_state(const std_msgs::msg::UInt8::SharedPtr m) {
  tcp_state_.store(m->data);
  // Optional link-state animation (raw one-byte command), if your lighting Arduino supports it.
  uint8_t light_cmd = 0;
  switch (m->data) {
    case 0: light_cmd = 11; break; // Disconnected -> StopConnecting
    case 1: light_cmd = 13; break; // Listening    -> Reconnecting
    case 2: light_cmd = 12; break; // Connected
  }
  if (light_cmd) {
    last_light_cmd_ = light_cmd;
    if (!send_light_cmd(light_cmd)) {
      RCLCPP_WARN(get_logger(), "Lighting I2C write failed for TCP state cmd (errno=%d: %s)", errno, std::strerror(errno));
    }
  }
}

// Command builder (unchanged)
DriveCommandWire I2cBridgeNode::build_drive_command() {
  DriveCommandWire dc{};
  const bool estop  = estop_.load();
  const bool tcp_ok = (tcp_state_.load() == 2);

  bool enable = (!estop) && (tcp_ok || default_enable_);
  dc.flags = 0;
  if (enable) dc.flags |= 0x01;
  if (estop)  dc.flags |= 0x02;

  int16_t speed = 0;
  int16_t steer = 0;
  uint8_t brake = 0;

  if (last_target_) {
    brake = std::min<uint8_t>(last_target_->brake_percent, 100);
    speed = last_target_->target_speed_mmps;
    steer = last_target_->target_steer_millirad;
    if (speed < 0) dc.flags |= 0x04; // Reverse flag for your MCU (optional)
  }

  dc.brake_percent         = brake;
  dc.target_speed_mmps     = speed;
  dc.target_steer_millirad = steer;

  maybe_auto_brake_lights(brake);
  return dc;
}

// Periodic
void I2cBridgeNode::tick_drive_cmd() {
  using clock = std::chrono::steady_clock;
  static auto prev = clock::now();
  auto start = clock::now();

  DriveCommandWire dc = build_drive_command();

  auto w0 = clock::now();
  bool ok = drive.sendCommand(dc);
  auto w1 = clock::now();

  auto period_gap = std::chrono::duration_cast<std::chrono::milliseconds>(start - prev).count();
  auto write_ms   = std::chrono::duration_cast<std::chrono::milliseconds>(w1 - w0).count();
  prev = start;

  if (period_gap > 100 || write_ms > 50 || !ok) {
    RCLCPP_WARN(get_logger(), "TX gap=%ld ms write=%ld ms ok=%d errno=%d (%s)",
                (long)period_gap, (long)write_ms, (int)ok, errno, std::strerror(errno));
  }
}

void I2cBridgeNode::tick_drive_status() {
  DriveStatusWire st{};

  if (!drive.readStatus(st)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Drive status read failed (errno=%d: %s)", errno, std::strerror(errno));
    return;
  }

  auto now = this->get_clock()->now();

  vehicle_core::msg::DriveFeedback fb;
  fb.speed_mmps     = st.measured_speed_mmps;
  fb.steer_millirad = st.measured_steer_millirad;
  pub_drive_feedback_->publish(fb);
  geometry_msgs::msg::TwistWithCovarianceStamped v;
  v.header.stamp = now;
  v.header.frame_id = "base_link";
  v.twist.twist.linear.x = 0.001 * fb.speed_mmps; // convert mm/s -> m/s
  v.twist.covariance[0] = 0.15 * 0.15;            // vx variance
  pub_velocity_->publish(v);

  vehicle_core::msg::DriveStatus ds;
  ds.gear       = current_gear();
  ds.braking    = current_braking(last_target_ ? last_target_->brake_percent : 0);
  ds.fault_bits = st.fault_bits;
  pub_drive_status_->publish(ds);

  if (st.fault_bits != last_fault_bits_) {
    last_fault_bits_ = st.fault_bits;
    std_msgs::msg::UInt16 f; f.data = st.fault_bits;
    pub_drive_fault_bits_->publish(f);
  }
}

void I2cBridgeNode::tick_imu_data_() {
  double qw=0,qx=0,qy=0,qz=0, gx=0,gy=0,gz=0, ax=0,ay=0,az=0, mx=0,my=0,mz=0;
  uint8_t c_sys=0,c_g=0,c_a=0,c_m=0;
  bool ok = true;

  ok &= imu.read_quat(qw,qx,qy,qz);
  ok &= imu.read_gyro_rad_s(gx,gy,gz);
  ok &= imu.read_lin_acc_ms2(ax,ay,az);
  ok &= imu.read_mag_uT(mx,my,mz);
  ok &= imu.read_calib(c_sys,c_g,c_a,c_m);
  
  if (!ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU read failed: %s", imu.getLastError().c_str());
    // your existing “status only” path
    vehicle_core::msg::IMUStatus imu_status{};
    imu_status.header.stamp = this->get_clock()->now();
    imu_status.header.frame_id = "imu_link";
    imu_status.calib_sys   = c_sys;
    imu_status.calib_gyro  = c_g;
    imu_status.calib_accel = c_a;
    imu_status.calib_mag   = c_m;
    imu_status.imu_data_ok = false;
    pub_imu_status_->publish(imu_status);
    return;
  }

  auto now = this->get_clock()->now();
  const std::string& frame = "imu_link"; // e.g., member set to "imu_link"

  // Normalize quaternion & sanity check
  {
    const double n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n > 1e-6 && std::isfinite(n)) {
      qw/=n; qx/=n; qy/=n; qz/=n;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU quat invalid; skipping publish");
      return;
    }
  }

  // --- IMU ---
  sensor_msgs::msg::Imu imu_msg{};
  imu_msg.header.stamp = now;
  imu_msg.header.frame_id = frame;

  imu_msg.orientation.w = qw;
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;

  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  // Covariances: provide usable defaults for EKF (tune later)
  imu_msg.orientation_covariance[0] = 7.6e-5;  // roll
  imu_msg.orientation_covariance[4] = 7.6e-5;  // pitch
  imu_msg.orientation_covariance[8] = 7.6e-5;  // yaw

  imu_msg.angular_velocity_covariance[0] = 1e-4;
  imu_msg.angular_velocity_covariance[4] = 1e-4;
  imu_msg.angular_velocity_covariance[8] = 1e-4;

  imu_msg.linear_acceleration_covariance[0] = 1e-2;
  imu_msg.linear_acceleration_covariance[4] = 1e-2;
  imu_msg.linear_acceleration_covariance[8] = 1e-2;

  pub_imu_data_->publish(imu_msg);

  // --- Magnetometer (Tesla) ---
  sensor_msgs::msg::MagneticField mag_msg{};
  mag_msg.header.stamp = now;
  mag_msg.header.frame_id = frame;
  mag_msg.magnetic_field.x = mx * 1e-6;
  mag_msg.magnetic_field.y = my * 1e-6;
  mag_msg.magnetic_field.z = mz * 1e-6;
  // Covariance unknown for now
  mag_msg.magnetic_field_covariance[0] = -1.0;
  pub_imu_mag_->publish(mag_msg);

  // --- Status ---
  vehicle_core::msg::IMUStatus imu_status{};
  imu_status.header.stamp = now;
  imu_status.header.frame_id = frame;
  imu_status.calib_sys   = c_sys;
  imu_status.calib_gyro  = c_g;
  imu_status.calib_accel = c_a;
  imu_status.calib_mag   = c_m;
  imu_status.imu_data_ok = true;
  pub_imu_status_->publish(imu_status);

  if (mag_csv_opened_) {
    const double t = now.seconds();  // ROS2 rclcpp::Time -> double seconds
    // level-only heading (no tilt compensation)
    double heading = std::atan2(mx * 1e-6, my * 1e-6) * 180.0 / M_PI;
    if (heading < 0) heading += 360.0;

    mag_csv_ << std::fixed << std::setprecision(6) << t << ','
            << std::scientific << std::setprecision(9)
            << (mx * 1e-6) << ',' << (my * 1e-6) << ',' << (mz * 1e-6) << ','
            << qw << ',' << qx << ',' << qy << ',' << qz << ','
            << std::fixed << std::setprecision(3) << heading
            << '\n';
    static size_t n = 0;
    if ((++n % 20) == 0) mag_csv_.flush();
  }
}

// Utilities
void I2cBridgeNode::maybe_auto_brake_lights(uint8_t brake_percent) {
  const bool on = (brake_percent >= brake_light_threshold_);
  const uint8_t cmd_on  = 9;   // BrakeLightsOn (per your enum)
  const uint8_t cmd_off = 10;  // BrakeLightsOff
  const uint8_t want = on ? cmd_on : cmd_off;

  std::lock_guard<std::mutex> lks(lights_mtx_);
  if (lights_.brake == on) return; // no change
  if (send_light_cmd(want)) {
    lights_.brake = on;
    publish_lights_status_locked();
    last_light_cmd_ = want;
  } else {
    RCLCPP_WARN(get_logger(), "Brake light I2C write failed (errno=%d: %s)", errno, std::strerror(errno));
  }
}

bool I2cBridgeNode::send_light_cmd(uint8_t cmd_byte) {
  return lighting.sendCommand(cmd_byte);
}

void I2cBridgeNode::publish_lights_status_locked() {
  vehicle_core::msg::LightsStatus msg;
  msg.brake_lights  = lights_.brake     ? 1 : 0;
  msg.reverse_lights= lights_.reverse   ? 1 : 0;
  msg.right_sig     = lights_.right_sig ? 1 : 0;
  msg.left_sig      = lights_.left_sig  ? 1 : 0;
  msg.headlights    = lights_.headlights? 1 : 0;
  msg.hazards       = lights_.hazards   ? 1 : 0;
  pub_lights_status_->publish(msg);
}

uint8_t I2cBridgeNode::current_gear() const {
  if (!last_target_) return 0; // Neutral/Coast
  const auto s = last_target_->target_speed_mmps;
  if (s > 0) return 2; // Forward
  if (s < 0) return 3; // Reverse
  return 1;            // Coast
}

bool I2cBridgeNode::current_braking(uint8_t brake_percent) const {
  return brake_percent >= static_cast<uint8_t>(std::clamp(brake_light_threshold_, 0, 100));
}

} // namespace vehicle_core

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_core::I2cBridgeNode)

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <regex>
#include <unordered_map>
#include <fstream>

// ROS2 messages
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "BaseToCar.pb.h"
#include "VehicleData.pb.h"
#include "PerceptionData.pb.h"
#include "TrajectoryData.pb.h"
#include "UdpMessage.pb.h"

// UDP stuff
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

// using namespace boost::asio;

// using namespace std::chrono_literals;

class Telemetry : public rclcpp::Node
{
  public:
    Telemetry(): 
    	Node("telemetry"), 
     	basestation_socket(io_service_main), 
     	racecar_socket(io_service_main)
    {

      // setup QOS to be best effort
      rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos.best_effort();

      //Get Parameters 
      std::string joystick_command_topic_;
      this->declare_parameter<std::string>("joystick_command_topic", "joystick/command");
      this->get_parameter("joystick_command_topic", joystick_command_topic_);

      this->declare_parameter<std::string>("basestation_ip", "192.168.0.201");
      this->get_parameter<std::string>("basestation_ip", basestation_ip);

      this->declare_parameter<std::string>("racecar_ip", "192.168.0.200");
      this->get_parameter<std::string>("racecar_ip", racecar_ip);

      this->declare_parameter<std::uint16_t>("basestation_port", 23431);
      this->get_parameter<std::uint16_t>("basestation_port", basestation_port);

      this->declare_parameter<std::uint16_t>("racecar_port", 23531);
      this->get_parameter<std::uint16_t>("racecar_port", racecar_port);

      this->declare_parameter<bool>("joystick", false);
      this->get_parameter<bool>("joystick", joystick_enable);

      this->declare_parameter<float>("steering_ratio", 15.0);
      this->get_parameter<float>("steering_ratio", steering_ratio);

      // ROS2 topics which should be republished
      sub_ct_report = this->create_subscription<deep_orange_msgs::msg::CtReport>("raptor_dbw_interface/ct_report", 1, std::bind(&Telemetry::ct_report_callback, this, std::placeholders::_1));
      sub_pt_report = this->create_subscription<deep_orange_msgs::msg::PtReport>("raptor_dbw_interface/pt_report", 1, std::bind(&Telemetry::pt_report_callback, this, std::placeholders::_1));
      sub_misc_report_do = this->create_subscription<deep_orange_msgs::msg::MiscReport>("raptor_dbw_interface/misc_report_do", 1, std::bind(&Telemetry::misc_report_do_callback, this, std::placeholders::_1));
      sub_rc_to_ct = this->create_subscription<deep_orange_msgs::msg::RcToCt>("vehicle/rc_to_ct_info", 1, std::bind(&Telemetry::rc_to_ct_callback, this, std::placeholders::_1));
      sub_ws_report = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>("raptor_dbw_interface/wheel_speed_report", 1, std::bind(&Telemetry::ws_report_callback, this, std::placeholders::_1));
      sub_lateral_error = this->create_subscription<std_msgs::msg::Float32>("lateral_error", 1, std::bind(&Telemetry::lateral_error_callback, this, std::placeholders::_1));
      sub_steering_cmd = this->create_subscription<raptor_dbw_msgs::msg::SteeringCmd>("steering_cmd", 1, std::bind(&Telemetry::steer_cmd_callback, this, std::placeholders::_1));
      sub_mpc_hz = this->create_subscription<std_msgs::msg::Float32>("mpc/steering_vel", 1, std::bind(&Telemetry::mpc_hz_callback, this, std::placeholders::_1));
      sub_stop_reason = this->create_subscription<std_msgs::msg::String>("graceful_stop/reason", 1, std::bind(&Telemetry::reason_callback, this, std::placeholders::_1));
      sub_novatel = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>("novatel_bottom/bestpos", 1, std::bind(&Telemetry::gps_callback, this, std::placeholders::_1));
      sub_desvel = this->create_subscription<std_msgs::msg::Float32>("desired_velocity_readout", 1, std::bind(&Telemetry::desvel_callback, this, std::placeholders::_1));
      sub_path = this->create_subscription<std_msgs::msg::String>("path_name", 1, std::bind(&Telemetry::path_callback, this, std::placeholders::_1));
      // base pose on rl
      sub_base_pose_rl = this->create_subscription<geometry_msgs::msg::PointStamped>("pure_pursuit/base_projection", 1, std::bind(&Telemetry::base_pose_callback, this, std::placeholders::_1));
      // target pose on rl
      sub_target_pose_rl = this->create_subscription<geometry_msgs::msg::PointStamped>("pure_pursuit/target_point", 1, std::bind(&Telemetry::target_pose_callback, this, std::placeholders::_1));
      // steering pose
      sub_vehicle_odom = this->create_subscription<nav_msgs::msg::Odometry>("vehicle_odom", 1, std::bind(&Telemetry::vehicle_odom_callback, this, std::placeholders::_1));
      sub_clustering_hb = this->create_subscription<std_msgs::msg::Bool>("heartbeat/clustering", 1, std::bind(&Telemetry::clustering_hb_callback, this, std::placeholders::_1));
      sub_radar_hb = this->create_subscription<std_msgs::msg::Bool>("heartbeat/radar", 1, std::bind(&Telemetry::radar_hb_callback, this, std::placeholders::_1));
      sub_dnn_hb = this->create_subscription<std_msgs::msg::Bool>("heartbeat/dnn", 1, std::bind(&Telemetry::dnn_hb_callback, this, std::placeholders::_1));
      sub_ekf_hb = this->create_subscription<std_msgs::msg::Bool>("heartbeat/ekf_health", 1, std::bind(&Telemetry::ekf_hb_callback, this, std::placeholders::_1));
      sub_tire_report = this->create_subscription<deep_orange_msgs::msg::TireReport>("raptor_dbw_interface/tire_report", 1, std::bind(&Telemetry::tire_report_callback, this, std::placeholders::_1));
      sub_brake_cmd = this->create_subscription<raptor_dbw_msgs::msg::BrakeCmd>("brake_cmd", 1, std::bind(&Telemetry::brake_cmd_callback, this, std::placeholders::_1));
      sub_accelerator_cmd = this->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>("accelerator_cmd", 1, std::bind(&Telemetry::accelerator_cmd_callback, this, std::placeholders::_1));
      sub_gear_cmd = this->create_subscription<std_msgs::msg::UInt8>("gear_cmd", 1, std::bind(&Telemetry::gear_cmd_callback, this, std::placeholders::_1));
      sub_ekf = this->create_subscription<nav_msgs::msg::Odometry>("target_vehicle/ekf_odometry", qos, std::bind(&Telemetry::ekf_callback, this, std::placeholders::_1));
      sub_radar = this->create_subscription<nav_msgs::msg::Odometry>("radar_detections_healthy", qos, std::bind(&Telemetry::radar_callback, this, std::placeholders::_1));
      sub_clustering = this->create_subscription<nav_msgs::msg::Odometry>("lidar_detections_clustering_healthy", qos, std::bind(&Telemetry::clustering_callback, this, std::placeholders::_1));
      sub_dnn = this->create_subscription<nav_msgs::msg::Odometry>("lidar_detections_dnn_healthy", qos, std::bind(&Telemetry::dnn_callback, this, std::placeholders::_1));
      sub_ghost = this->create_subscription<nav_msgs::msg::Odometry>("planner/ghost_obs", qos, std::bind(&Telemetry::ghost_callback, this, std::placeholders::_1));
      sub_ref_traj = this->create_subscription<sensor_msgs::msg::PointCloud2>("path_server/reference", 1, std::bind(&Telemetry::ref_traj_callback, this, std::placeholders::_1));
      sub_feasible_traj = this->create_subscription<sensor_msgs::msg::PointCloud2>("planner/feasible_trajectory", 1, std::bind(&Telemetry::feasible_traj_callback, this, std::placeholders::_1));
      sub_opp_traj = this->create_subscription<sensor_msgs::msg::PointCloud2>("opp_trajectory", 1, std::bind(&Telemetry::opp_traj_callback, this, std::placeholders::_1));
      sub_accel_filtered = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>("acceleration", 1, std::bind(&Telemetry::accel_filtered_callback, this, std::placeholders::_1));
      sub_lat_on = this->create_subscription<std_msgs::msg::Bool>("autonomy/lat_on", 1, std::bind(&Telemetry::lat_on_callback, this, std::placeholders::_1));
      sub_long_on = this->create_subscription<std_msgs::msg::Bool>("autonomy/long_on", 1, std::bind(&Telemetry::long_on_callback, this, std::placeholders::_1));
      sub_can_overtake = this->create_subscription<std_msgs::msg::Bool>("planner/enable", 1, std::bind(&Telemetry::can_overtake_callback, this, std::placeholders::_1));
      sub_planner_action = this->create_subscription<std_msgs::msg::String>("planner/action", 1, std::bind(&Telemetry::planner_action_callback, this, std::placeholders::_1));
      sub_planner_state = this->create_subscription<std_msgs::msg::Int32>("planner/race_state", 1, std::bind(&Telemetry::planner_state_callback, this, std::placeholders::_1));
      sub_overtake_state = this->create_subscription<std_msgs::msg::Int32>("planner/overtake_state", 1, std::bind(&Telemetry::overtake_state_callback, this, std::placeholders::_1));
      sub_followd_feedback = this->create_subscription<std_msgs::msg::Float32>("planner/follow_d_feeback", 1, std::bind(&Telemetry::followd_feedback_callback, this, std::placeholders::_1));
      sub_planner_controlvel = this->create_subscription<std_msgs::msg::Float32>("planner/control_vel", 1, std::bind(&Telemetry::planner_controlvel_callback, this, std::placeholders::_1));
      sub_opponent_dist = this->create_subscription<std_msgs::msg::Float32>("planner/opponent_distance", 1, std::bind(&Telemetry::opponent_dist_callback, this, std::placeholders::_1));
      sub_planner_status = this->create_subscription<std_msgs::msg::Bool>("planner/status", 1, std::bind(&Telemetry::planner_status_callback, this, std::placeholders::_1));
      sub_planner_debug_state = this->create_subscription<std_msgs::msg::String>("planner/state", 1, std::bind(&Telemetry::planner_debug_state_callback, this, std::placeholders::_1));
      sub_using_mixnet = this->create_subscription<std_msgs::msg::Bool>("using_mixnet", 1, std::bind(&Telemetry::using_mixnet_callback, this, std::placeholders::_1));
      sub_debug1 = this->create_subscription<std_msgs::msg::Float32>("mpc/steering_vel", 1, std::bind(&Telemetry::debug1_callback, this, std::placeholders::_1));
      //sub_debug2 = this->create_subscription<std_msgs::msg::Float32>("/debug2", 1, std::bind(&Telemetry::debug2_callback, this, std::placeholders::_1));
      //sub_debug3 = this->create_subscription<std_msgs::msg::Float32>("/debug3", 1, std::bind(&Telemetry::debug3_callback, this, std::placeholders::_1));
      sub_mpc_debugging = this->create_subscription<deep_orange_msgs::msg::MpcTuning>("mpc_debugging", 1, std::bind(&Telemetry::mpc_debugging_callback, this, std::placeholders::_1));
      sub_p_long_errors = this->create_subscription<std_msgs::msg::Float32MultiArray>("/trajectory_longitudinal_errors", 1, std::bind(&Telemetry::long_errors_callback, this, std::placeholders::_1));
      sub_p_lat_errors = this->create_subscription<std_msgs::msg::Float32MultiArray>("/trajectory_lateral_errors", 1, std::bind(&Telemetry::lat_errors_callback, this, std::placeholders::_1));
      // timer which handles sending packets
      send_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Telemetry::send_callback, this));
      // timer which handles receiving packets
      receive_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Telemetry::receive_callback, this));
      if(!joystick_enable){
        joystick_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Telemetry::spoof_joystick, this));
      }

      pub_joystick_command = this->create_publisher<deep_orange_msgs::msg::JoystickCommand>(joystick_command_topic_, 1);
      pub_desired_velocity = this->create_publisher<std_msgs::msg::Float32>("path_server/reference_velocity", 1);
      pub_switch_path = this->create_publisher<std_msgs::msg::String>("path_server/switch_path", 1);
      pub_ct_input = this->create_publisher<std_msgs::msg::Int32>("ct_input", 1);
      pub_enable_lon = this->create_publisher<std_msgs::msg::Bool>("enable_long", 1);
      pub_enable_lat = this->create_publisher<std_msgs::msg::Bool>("enable_lat", 1);
      pub_enable_mpc = this->create_publisher<std_msgs::msg::Bool>("mpc/enable", 1);
      pub_followd = this->create_publisher<std_msgs::msg::Float32>("planner/follow_d", 1);
      pub_spoof_flags = this->create_publisher<deep_orange_msgs::msg::RcToCt>("raptor_dbw_interface/rc_to_ct_bs", 1);

      // setup UDP interfaces
      basestation_socket.open(boost::asio::ip::udp::v4());
      basestation_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(basestation_ip), basestation_port);
      RCLCPP_INFO(this->get_logger(), "Established connection to send to remote at : %s:%u", basestation_ip.c_str(), basestation_port);

      racecar_socket.open(boost::asio::ip::udp::v4());
      racecar_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(racecar_ip), racecar_port);
      racecar_socket.bind(racecar_endpoint);
      racecar_socket.non_blocking(true);
      RCLCPP_INFO(this->get_logger(), "Established connection to receive from remote at : %s:%u", basestation_ip.c_str(), basestation_port);

      // Build string-to-int conversion map
      stops[""] = 0;
      build_hashmap(&stops, ament_index_cpp::get_package_share_directory("node_health_monitor")+"/src/node_health_monitor.cpp", "reason.data = \\\"(.+)\\\"");
      build_hashmap(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "pub_warning\\(\\\"(.+)\\\"\\)");
      build_hashmap(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "hstop_car\\(\\\"(.+)\\\"\\)");
      build_hashmap(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "gstop_car\\(\\\"(.+)\\\"\\)");
      build_hashmap(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "estop_car\\(\\\"(.+)\\\"\\)");
    }

  private:
    void send_callback()
    {  
      // Fill Vehicle Message
      VehicleData vehicle_message;

      // CT Report
      vehicle_message.set_track_flag_ack(msg_ct_report.track_flag_ack);
      vehicle_message.set_veh_flag_ack(msg_ct_report.veh_flag_ack);
      vehicle_message.set_ct_state(msg_ct_report.ct_state);

      // PT Report
      vehicle_message.set_fuel_pressure(msg_pt_report.fuel_pressure);
      vehicle_message.set_transmission_oil_temperature(msg_pt_report.transmission_oil_temperature);
      vehicle_message.set_engine_oil_temperature(msg_pt_report.engine_oil_temperature);
      vehicle_message.set_engine_coolant_temperature(msg_pt_report.engine_coolant_temperature);
      vehicle_message.set_engine_rpm(msg_pt_report.engine_rpm);
      vehicle_message.set_current_gear(msg_pt_report.current_gear);

      // Misc Report
      vehicle_message.set_battery_voltage(msg_misc_report_do.battery_voltage);
      vehicle_message.set_sys_state(msg_misc_report_do.sys_state);

      // Autonomy
      vehicle_message.set_lateral_error(msg_autonomy.lateral_error);
      vehicle_message.set_lat_stdev(msg_autonomy.lat_stdev);
      vehicle_message.set_lon_stdev(msg_autonomy.lon_stdev);
      vehicle_message.set_steering_cmd(msg_autonomy.steering_cmd);
      vehicle_message.set_desired_velocity_readout(msg_autonomy.desired_velocity_readout);
      vehicle_message.set_current_velocity(msg_autonomy.current_velocity);
      vehicle_message.set_target_speed(msg_autonomy.target_speed);
      vehicle_message.set_brake_cmd(msg_autonomy.brake_cmd);
      vehicle_message.set_accelerator_cmd(msg_autonomy.accelerator_cmd);
      vehicle_message.set_gear_cmd(msg_autonomy.gear_cmd);
      vehicle_message.set_can_overtake(msg_autonomy.can_overtake);

      // Convert strings to ints to save space
      int stringint;
      stringint = stops[msg_autonomy.stop_reason]; // If message isn't in stops, defaults to 0
      if (stringint == 0 && msg_autonomy.stop_reason != "") {
        vehicle_message.set_stop_reason(stops.size()); // Unknown/Malformed Error
      } else {
        vehicle_message.set_stop_reason(stringint);
      }
      stringint = paths[msg_autonomy.path_name];
      if (stringint == 0 && msg_autonomy.path_name != "") {
        vehicle_message.set_path_name(paths.size());
      } else {
        vehicle_message.set_path_name(stringint);
      }
      stringint = actions[msg_autonomy.planner_output];
      if (stringint == 0 && msg_autonomy.planner_output != "") {
        vehicle_message.set_planner_action(actions.size());
      } else {
        vehicle_message.set_planner_action(stringint);
      }
      /*stringint = states[msg_autonomy.planner_state];
      if (stringint == 0 && msg_autonomy.planner_state != "") {
        vehicle_message.set_planner_state(states.size());
      } else {
        vehicle_message.set_planner_state(stringint);
      }*/
      vehicle_message.set_planner_state(msg_autonomy.planner_state);
      vehicle_message.set_overtake_state(msg_autonomy.overtake_state);

      // Vehicle position, angle, steering
      vehicle_message.set_veh_x(msg_vehicle_pose.pose.pose.position.x);
      vehicle_message.set_veh_y(msg_vehicle_pose.pose.pose.position.y);
      float veh_ang = atan2(
          2.0*(msg_vehicle_pose.pose.pose.orientation.x*msg_vehicle_pose.pose.pose.orientation.y + msg_vehicle_pose.pose.pose.orientation.w*msg_vehicle_pose.pose.pose.orientation.z),
          msg_vehicle_pose.pose.pose.orientation.w*msg_vehicle_pose.pose.pose.orientation.w + msg_vehicle_pose.pose.pose.orientation.x*msg_vehicle_pose.pose.pose.orientation.x - msg_vehicle_pose.pose.pose.orientation.y*msg_vehicle_pose.pose.pose.orientation.y - msg_vehicle_pose.pose.pose.orientation.z*msg_vehicle_pose.pose.pose.orientation.z
          );
      vehicle_message.set_veh_ang(veh_ang);
      vehicle_message.set_target_point_x(msg_target_pose.point.x);
      vehicle_message.set_target_point_y(msg_target_pose.point.y);
      vehicle_message.set_base_pose_x(msg_base_pose.point.x);
      vehicle_message.set_base_pose_y(msg_base_pose.point.y);

      // Vehicle acceleration
      vehicle_message.set_accel_x(msg_accel_filtered.accel.accel.linear.x);
      vehicle_message.set_accel_y(msg_accel_filtered.accel.accel.linear.y);

      // Vehicle Autonomy indicators
      vehicle_message.set_lat_on(lat_on);
      vehicle_message.set_long_on(long_on);

      // Tire Data
      if (msg_tire_report.fl_tire_temperature.size() > 0) {
        float avg = 0.0;
        for (unsigned int i = 0; i < msg_tire_report.fl_tire_temperature.size(); i++) {
          avg+=msg_tire_report.fl_tire_temperature[i];
        }
        vehicle_message.set_fl_tire_temperature(avg/msg_tire_report.fl_tire_temperature.size());
      }
      if (msg_tire_report.fr_tire_temperature.size() > 0) {
        float avg = 0.0;
        for (unsigned int i = 0; i < msg_tire_report.fr_tire_temperature.size(); i++) {
          avg+=msg_tire_report.fr_tire_temperature[i];
        }
        vehicle_message.set_fr_tire_temperature(avg/msg_tire_report.fr_tire_temperature.size());
      }
      if (msg_tire_report.rl_tire_temperature.size() > 0) {
        float avg = 0.0;
        for (unsigned int i = 0; i < msg_tire_report.rl_tire_temperature.size(); i++) {
          avg+=msg_tire_report.rl_tire_temperature[i];
        }
        vehicle_message.set_rl_tire_temperature(avg/msg_tire_report.rl_tire_temperature.size());
      }
      if (msg_tire_report.rr_tire_temperature.size() > 0) {
        float avg = 0.0;
        for (unsigned int i = 0; i < msg_tire_report.rr_tire_temperature.size(); i++) {
          avg+=msg_tire_report.rr_tire_temperature[i];
        }
        vehicle_message.set_rr_tire_temperature(avg/msg_tire_report.rr_tire_temperature.size());
      }

      // Planner Debug Data
      vehicle_message.set_planner_maxspeed(planner_maxspeed);
      vehicle_message.set_planner_zone(planner_zone);
      vehicle_message.set_planner_startzone(planner_startzone);
      vehicle_message.set_planner_controlvel(planner_controlvel);
      vehicle_message.set_opponent_dist(opponent_dist);
      vehicle_message.set_planner_status(planner_status);

      // Other misc vehicle data
      vehicle_message.set_using_mixnet(using_mixnet);

      vehicle_message.set_debug1(debug1);
      //vehicle_message.set_debug2(debug2);
      //vehicle_message.set_debug3(debug3);
      
      if (steer_count == 0) {
        vehicle_message.set_mpc_rate(0.0);
      } else {
        vehicle_message.set_mpc_rate(steer_count/steer_hz);
      }
      steer_count = 0;
      steer_hz = 0.0;

      vehicle_message.set_mpc_tracking_cost(mpc_tracking_cost);
      vehicle_message.set_mpc_heading_cost(mpc_heading_cost);
      vehicle_message.set_mpc_input_cost(mpc_input_cost);
      vehicle_message.set_mpc_rho(mpc_rho);
      vehicle_message.set_mpc_rhoq(mpc_rhoQ);
      vehicle_message.set_mpc_q_track(mpc_Q_track);
      vehicle_message.set_mpc_q_heading(mpc_Q_heading);
      vehicle_message.set_mpc_heading_error(mpc_heading_error);
      vehicle_message.set_mpc_solution_status(mpc_solution_status);

      for (int i = 0; i < 2*error_size; i++) {
        vehicle_message.add_perception_history(perception_errors[i]);
      }

      udp_message.clear_udp_data();
      udp_message.mutable_vehicle_data()->CopyFrom(vehicle_message);
      send_message(&udp_message); // Send Vehicle Message

      // Fill Perception Message
      PerceptionData perception_message;
      rclcpp::Time curr_time = rclcpp::Clock().now();

      // EKF detection
      Detection *ekf_detection = perception_message.add_detections();
      ekf_detection->set_id(Detection_DetectionId_EKF);

      double ekf_hb_dt = static_cast<double>((curr_time - ekf_hb_time).seconds());
      //if (msg_ekf.pose.covariance[0] >= 3.1623 or msg_ekf.pose.covariance[7] >= 3.1623) { //sqrt(10)
      if (msg_ekf.pose.covariance[0] >= 100 or msg_ekf.pose.covariance[7] >= 100) { // Temporary values
        ekf_detection->set_source_died(true);
      } else if (ekf_hb_dt > 1.0) {
        ekf_detection->set_source_died(true);
      } else {
        ekf_detection->set_source_died(msg_autonomy.ekf_died);
      }

      ekf_detection->set_center_x(msg_ekf.pose.pose.position.x);
      ekf_detection->set_center_y(msg_ekf.pose.pose.position.y);
      ekf_detection->set_velocity(msg_ekf.twist.twist.linear.x);
      float ekf_ang = atan2(
          2.0*(msg_ekf.pose.pose.orientation.x*msg_ekf.pose.pose.orientation.y + msg_ekf.pose.pose.orientation.w*msg_ekf.pose.pose.orientation.z),
          msg_ekf.pose.pose.orientation.w*msg_ekf.pose.pose.orientation.w + msg_ekf.pose.pose.orientation.x*msg_ekf.pose.pose.orientation.x - msg_ekf.pose.pose.orientation.y*msg_ekf.pose.pose.orientation.y - msg_ekf.pose.pose.orientation.z*msg_ekf.pose.pose.orientation.z
          );
      ekf_detection->set_angle(ekf_ang);
      double ekf_dt = static_cast<double>((curr_time - ekf_time).seconds());
      ekf_detection->set_fresh(ekf_dt < 3.0);

      // Radar detection
      Detection *radar_detection = perception_message.add_detections();
      radar_detection->set_id(Detection_DetectionId_RADAR);

      double radar_hb_dt = static_cast<double>((curr_time - radar_hb_time).seconds());
      if (radar_hb_dt > 1.0) {
        radar_detection->set_source_died(true);
      } else {
        radar_detection->set_source_died(msg_autonomy.radar_died);
      }

      radar_detection->set_center_x(msg_radar.pose.pose.position.x);
      radar_detection->set_center_y(msg_radar.pose.pose.position.y);
      radar_detection->set_velocity(msg_radar.twist.twist.linear.x);
      float radar_ang = atan2(
          2.0*(msg_radar.pose.pose.orientation.x*msg_radar.pose.pose.orientation.y + msg_radar.pose.pose.orientation.w*msg_radar.pose.pose.orientation.z),
          msg_radar.pose.pose.orientation.w*msg_radar.pose.pose.orientation.w + msg_radar.pose.pose.orientation.x*msg_radar.pose.pose.orientation.x - msg_radar.pose.pose.orientation.y*msg_radar.pose.pose.orientation.y - msg_radar.pose.pose.orientation.z*msg_radar.pose.pose.orientation.z
          );
      radar_detection->set_angle(radar_ang);
      double radar_dt = static_cast<double>((curr_time - radar_time).seconds());
      radar_detection->set_fresh(radar_dt < 3.0);

      // Ghost detection
      Detection *ghost_detection = perception_message.add_detections();
      ghost_detection->set_id(Detection_DetectionId_GHOST);

      ghost_detection->set_center_x(msg_ghost.pose.pose.position.x);
      ghost_detection->set_center_y(msg_ghost.pose.pose.position.y);
      ghost_detection->set_velocity(msg_ghost.twist.twist.linear.x);
      float ghost_ang = atan2(
          2.0*(msg_ghost.pose.pose.orientation.x*msg_ghost.pose.pose.orientation.y + msg_ghost.pose.pose.orientation.w*msg_ghost.pose.pose.orientation.z),
          msg_ghost.pose.pose.orientation.w*msg_ghost.pose.pose.orientation.w + msg_ghost.pose.pose.orientation.x*msg_ghost.pose.pose.orientation.x - msg_ghost.pose.pose.orientation.y*msg_ghost.pose.pose.orientation.y - msg_ghost.pose.pose.orientation.z*msg_ghost.pose.pose.orientation.z
          );
      ghost_detection->set_angle(ghost_ang);
      double ghost_dt = static_cast<double>((curr_time - ghost_time).seconds());
      ghost_detection->set_fresh(ghost_dt < 3.0);

      // DNN detection
      Detection *dnn_detection = perception_message.add_detections();
      dnn_detection->set_id(Detection_DetectionId_DNN);

      double dnn_hb_dt = static_cast<double>((curr_time - dnn_hb_time).seconds());
      if (dnn_hb_dt > 1.0) {
        dnn_detection->set_source_died(true);
      } else {
        dnn_detection->set_source_died(msg_autonomy.dnn_died);
      }

      dnn_detection->set_center_x(msg_dnn.pose.pose.position.x);
      dnn_detection->set_center_y(msg_dnn.pose.pose.position.y);
      dnn_detection->set_velocity(msg_dnn.twist.twist.linear.x);
      float dnn_ang = atan2(
          2.0*(msg_dnn.pose.pose.orientation.x*msg_dnn.pose.pose.orientation.y + msg_dnn.pose.pose.orientation.w*msg_dnn.pose.pose.orientation.z),
          msg_dnn.pose.pose.orientation.w*msg_dnn.pose.pose.orientation.w + msg_dnn.pose.pose.orientation.x*msg_dnn.pose.pose.orientation.x - msg_dnn.pose.pose.orientation.y*msg_dnn.pose.pose.orientation.y - msg_dnn.pose.pose.orientation.z*msg_dnn.pose.pose.orientation.z
          );
      dnn_detection->set_angle(dnn_ang);
      double dnn_dt = static_cast<double>((curr_time - dnn_time).seconds());
      dnn_detection->set_fresh(dnn_dt < 3.0);

      // Clustering detection
      Detection *clustering_detection = perception_message.add_detections();
      clustering_detection->set_id(Detection_DetectionId_CLUSTERING);

      double clustering_hb_dt = static_cast<double>((curr_time - clustering_hb_time).seconds());
      if (clustering_hb_dt > 1.0) {
        clustering_detection->set_source_died(true);
      } else {
        clustering_detection->set_source_died(msg_autonomy.clustering_died);
      }

      clustering_detection->set_center_x(msg_clustering.pose.pose.position.x);
      clustering_detection->set_center_y(msg_clustering.pose.pose.position.y);
      clustering_detection->set_velocity(msg_clustering.twist.twist.linear.x);
      float clustering_ang = atan2(
          2.0*(msg_clustering.pose.pose.orientation.x*msg_clustering.pose.pose.orientation.y + msg_clustering.pose.pose.orientation.w*msg_clustering.pose.pose.orientation.z),
          msg_clustering.pose.pose.orientation.w*msg_clustering.pose.pose.orientation.w + msg_clustering.pose.pose.orientation.x*msg_clustering.pose.pose.orientation.x - msg_clustering.pose.pose.orientation.y*msg_clustering.pose.pose.orientation.y - msg_clustering.pose.pose.orientation.z*msg_clustering.pose.pose.orientation.z
          );
      clustering_detection->set_angle(clustering_ang);
      double clustering_dt = static_cast<double>((curr_time - clustering_time).seconds());
      clustering_detection->set_fresh(clustering_dt < 3.0);

      udp_message.clear_udp_data();
      udp_message.mutable_perception_data()->CopyFrom(perception_message);
      send_message(&udp_message); // Send Perception Message

      // Fill Trajectory Message
      TrajectoryData trajectory_message;
      // Reference Trajectory
      Trajectory *ref_traj = trajectory_message.add_trajectories();
      ref_traj->set_id(Trajectory_TrajectoryId_REFERENCE);

      if (msg_ref_traj.height > 0 && msg_ref_traj.width > 0) {
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg_ref_traj, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg_ref_traj, "y");
        int count = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
          if (count++%5 == 0) {
            ref_traj->add_path_x(*iter_x);
            ref_traj->add_path_y(*iter_y);
          }
        }
      }

      // Feasible (Planner) Trajectory
      Trajectory *feas_traj = trajectory_message.add_trajectories();
      feas_traj->set_id(Trajectory_TrajectoryId_FEASIBLE);

      double feas_traj_dt = static_cast<double>((curr_time - feas_traj_time).seconds());
      feas_traj->set_fresh(feas_traj_dt < 3.0);

      if (msg_feasible_traj.height > 0 && msg_feasible_traj.width > 0) {
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg_feasible_traj, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg_feasible_traj, "y");
        int count = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
          if (count++%5 == 0) {
            feas_traj->add_path_x(*iter_x);
            feas_traj->add_path_y(*iter_y);
          }
        }
      }

      // Opponent Trajectory
      Trajectory *opp_traj = trajectory_message.add_trajectories();
      opp_traj->set_id(Trajectory_TrajectoryId_OPPONENT);

      double opp_traj_dt = static_cast<double>((curr_time - opp_traj_time).seconds());
      opp_traj->set_fresh(opp_traj_dt < 3.0);

      if (msg_opp_traj.height > 0 && msg_opp_traj.width > 0) {
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg_opp_traj, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg_opp_traj, "y");
        int count = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
          if (count++%5 == 0) {
            opp_traj->add_path_x(*iter_x);
            opp_traj->add_path_y(*iter_y);
          }
        }
      }

      udp_message.clear_udp_data();
      udp_message.mutable_trajectory_data()->CopyFrom(trajectory_message);
      send_message(&udp_message); // Send Trajectory Message
    }

    void receive_callback()
    {
      boost::system::error_code err;
      std::fill_n(read_buffer,2048,0);
      // read all messages from udp socket
      while (racecar_socket.receive_from(boost::asio::buffer(read_buffer), racecar_endpoint, 0, err) != 0) {
        recv_mutex.lock();
        google::protobuf::io::CodedInputStream coded_input_stream(read_buffer, 2048);
        uint32_t size = 0;
        if (!coded_input_stream.ReadVarint32(&size)) {
          return;
        }

        UdpMessage recv_basestation_buffer;
        recv_basestation_buffer.ParseFromCodedStream(&coded_input_stream);
        recv_mutex.unlock();

        if (recv_basestation_buffer.has_bst_data()) {
          BaseToCar* data = recv_basestation_buffer.mutable_bst_data();

          msg_joystick_command.counter = data->counter(); 
          msg_joystick_command.emergency_stop = data->emergency_stop(); 
          msg_joystick_command.joy_enable = data->joy_enable();
          msg_joystick_command.steering_cmd = data->steering_cmd(); 
          msg_joystick_command.brake_cmd = data->brake_cmd();
          msg_joystick_command.accelerator_cmd = data->accelerator_cmd(); 
          msg_joystick_command.gear_cmd = data->gear_cmd(); 

          msg_spoof_flags.track_flag = data->track_flag();
          msg_spoof_flags.veh_flag = data->veh_flag();
          msg_spoof_flags.target_speed = data->def_speed();

          // publish everything on ros2
          pub_joystick_command->publish(msg_joystick_command);
          pub_spoof_flags->publish(msg_spoof_flags);

          rclcpp::Time curr_time = rclcpp::Clock().now();
          double desired_velocity_dt = static_cast<double>((curr_time - desired_velocity_time).seconds());
          if (data->desired_velocity() >= 0.0 && desired_velocity_dt > single_message_timeout) {
            std_msgs::msg::Float32 msg_desired_velocity = std_msgs::msg::Float32(); 
            msg_desired_velocity.data = data->desired_velocity();
            pub_desired_velocity->publish(msg_desired_velocity);
            desired_velocity_time = rclcpp::Clock().now();
          }

          double switch_path_dt = static_cast<double>((curr_time - switch_path_time).seconds());
          if (data->switch_path().compare("") && switch_path_dt > single_message_timeout) {
            std_msgs::msg::String msg_switch_path = std_msgs::msg::String();
            msg_switch_path.data = data->switch_path();
            pub_switch_path->publish(msg_switch_path);
            switch_path_time = rclcpp::Clock().now();
          }

          double ct_input_dt = static_cast<double>((curr_time - ct_input_time).seconds());
          if (data->ct_input() >= 0 && ct_input_dt > single_message_timeout) {
            std_msgs::msg::Int32 msg_ct_input = std_msgs::msg::Int32();
            msg_ct_input.data = data->ct_input();
            pub_ct_input->publish(msg_ct_input);
            ct_input_time = rclcpp::Clock().now();
          }

          double enable_lon_dt = static_cast<double>((curr_time - enable_lon_time).seconds());
          if ((data->enable_lon() == 0 || data->enable_lon() == 1) && enable_lon_dt > single_message_timeout) {
            std_msgs::msg::Bool msg_enable_lon = std_msgs::msg::Bool();
            msg_enable_lon.data = data->enable_lon();
            pub_enable_lon->publish(msg_enable_lon);
            enable_lon_time = rclcpp::Clock().now();
          }

          double enable_lat_dt = static_cast<double>((curr_time - enable_lat_time).seconds());
          if ((data->enable_lat() == 0 || data->enable_lat() == 1) && enable_lat_dt > single_message_timeout) {
            std_msgs::msg::Bool msg_enable_lat = std_msgs::msg::Bool();
            msg_enable_lat.data = data->enable_lat();
            pub_enable_lat->publish(msg_enable_lat);
            enable_lat_time = rclcpp::Clock().now();
          }

          double enable_mpc_dt = static_cast<double>((curr_time - enable_mpc_time).seconds());
          if ((data->enable_mpc() == 0 || data->enable_mpc() == 1) && enable_mpc_dt > single_message_timeout) {
            std_msgs::msg::Bool msg_enable_mpc = std_msgs::msg::Bool();
            msg_enable_mpc.data = data->enable_mpc();
            pub_enable_mpc->publish(msg_enable_mpc);
            enable_mpc_time = rclcpp::Clock().now();
          }

          double followd_dt = static_cast<double>((curr_time - followd_time).seconds());
          if ((data->follow_dist() == 0 || data->follow_dist() == 1) && followd_dt > single_message_timeout) {
            std_msgs::msg::Float32 msg_followd = std_msgs::msg::Float32();
            if (data->follow_dist() == 0) {
              msg_followd.data = curr_followd-5.0;
            } else {
              msg_followd.data = curr_followd+5.0;
            }
            pub_followd->publish(msg_followd);
            followd_time = rclcpp::Clock().now();
          }
        }
      }
    }

    boost::system::error_code send_message(UdpMessage *message) {
      message->set_udp_end(true); // Important: if you don't set this flag you may lose data

      boost::system::error_code err;
      boost::asio::streambuf output_stream_buffer;
      std::ostream output_stream(&output_stream_buffer);
      {
        google::protobuf::io::OstreamOutputStream raw_output_stream(&output_stream);
        google::protobuf::io::CodedOutputStream coded_output_stream(&raw_output_stream);
        coded_output_stream.WriteVarint32(message->ByteSizeLong());
        message->SerializeToCodedStream(&coded_output_stream);
      } // IMPORTANT: In order to flush a CodedOutputStream it has to be deleted, otherwise a 0 byte package is sent
      basestation_socket.send_to(output_stream_buffer.data(), basestation_endpoint, 0, err);
      return err;
    }

    void build_hashmap(std::unordered_map<std::string,int> *map, std::string fname, std::string pattern) {
      std::regex r_pattern(pattern);
      std::smatch m;
      std::string line;
      std::ifstream myfile(fname);
      if (myfile.is_open()) {
        while (getline(myfile,line)) {
          if (regex_search(line, m, r_pattern)) {
            (*map)[m[1]] = map->size();
          }
        }
        myfile.close();
      }
    }

    void spoof_joystick()
    {
      msg_joystick_command.counter = joystick_counter; 
      msg_joystick_command.emergency_stop = 0; 
      msg_joystick_command.joy_enable = 0;
      msg_joystick_command.steering_cmd = 0.0; 
      msg_joystick_command.brake_cmd = 0.0;
      msg_joystick_command.accelerator_cmd = 0.0; 
      msg_joystick_command.gear_cmd = 0.0;
      pub_joystick_command->publish(msg_joystick_command);

      joystick_counter = (joystick_counter+1)%8;
    }

    void tire_report_callback(const deep_orange_msgs::msg::TireReport::SharedPtr msg)
    {
      msg_tire_report = *msg;
    }

    void brake_cmd_callback(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg)
    {
      msg_autonomy.brake_cmd = msg->pedal_cmd;
    }
    void accelerator_cmd_callback(const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg)
    {
      msg_autonomy.accelerator_cmd = msg->pedal_cmd;
    }
    void gear_cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
      msg_autonomy.gear_cmd = msg->data;
    }

    void clustering_hb_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      clustering_hb_time = rclcpp::Clock().now();
      msg_autonomy.clustering_died = msg->data;
    }

    void radar_hb_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      radar_hb_time = rclcpp::Clock().now();
      msg_autonomy.radar_died = msg->data;
    }

    void dnn_hb_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      dnn_hb_time = rclcpp::Clock().now();
      msg_autonomy.dnn_died = msg->data;
    }

    void ekf_hb_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      ekf_hb_time = rclcpp::Clock().now();
      msg_autonomy.ekf_died = msg->data;
    }

    void base_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      msg_base_pose = *msg;
    }

    void target_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      msg_target_pose = *msg;
    }

    void vehicle_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      msg_vehicle_pose = *msg;
    }

    void ct_report_callback(const deep_orange_msgs::msg::CtReport::SharedPtr msg)
    {
        msg_ct_report = *msg;
    }
    void pt_report_callback(const deep_orange_msgs::msg::PtReport::SharedPtr msg)
    {
        msg_pt_report = *msg;
    }
    void misc_report_do_callback(const deep_orange_msgs::msg::MiscReport::SharedPtr msg)
    {
        msg_misc_report_do = *msg;
    }
    void rc_to_ct_callback(const deep_orange_msgs::msg::RcToCt::SharedPtr msg)
    {
        msg_rc_to_ct = *msg;
        msg_autonomy.target_speed = msg_rc_to_ct.target_speed;
    }
    void lateral_error_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        msg_autonomy.lateral_error = msg->data;
    }

    void gps_callback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
    {
      msg_autonomy.lat_stdev = msg->lat_stdev;
      msg_autonomy.lon_stdev = msg->lon_stdev;
    }

    void steer_cmd_callback(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg)
    {
      msg_autonomy.steering_cmd = msg->angle_cmd/steering_ratio;
    }

    void mpc_hz_callback(const std_msgs::msg::Float32::SharedPtr /*msg*/)
    {
      rclcpp::Time curr_time = rclcpp::Clock().now();
      steer_count+=1;
      steer_hz+=(curr_time-steer_time).seconds();
      steer_time = curr_time;
    }

    void reason_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      msg_reason = *msg;
      msg_autonomy.stop_reason = msg_reason.data;
    }

    void desvel_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      msg_desvel = *msg;
      msg_autonomy.desired_velocity_readout = msg_desvel.data;
    }

    void path_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      msg_path = *msg;
      msg_autonomy.path_name = msg_path.data;
    }

    void ws_report_callback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
    {
      msg_ws = *msg;
      msg_autonomy.current_velocity = (0.62137)*(1.0/2.0)*(msg_ws.rear_right + msg_ws.rear_left);
    }

    void ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      ekf_time = rclcpp::Clock().now();
      msg_ekf = *msg;
    }

    void radar_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      radar_time = rclcpp::Clock().now();
      msg_radar = *msg;
    }

    void clustering_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      clustering_time = rclcpp::Clock().now();
      msg_clustering = *msg;
    }

    void dnn_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      dnn_time = rclcpp::Clock().now();
      msg_dnn = *msg;
    }

    void ghost_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      ghost_time = rclcpp::Clock().now();
      msg_ghost = *msg;
    }

    void ref_traj_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      msg_ref_traj = *msg;
    }

    void feasible_traj_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      msg_feasible_traj = *msg;
      feas_traj_time = rclcpp::Clock().now();
    }

    void opp_traj_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      msg_opp_traj = *msg;
      opp_traj_time = rclcpp::Clock().now();
    }

    void accel_filtered_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
    {
      msg_accel_filtered = *msg;
    }

    void lat_on_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      lat_on = msg->data;
    }

    void long_on_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      long_on = msg->data;
    }

    void can_overtake_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      msg_autonomy.can_overtake = msg->data;
    }

    void planner_action_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      msg_autonomy.planner_output = msg->data;
    }

    void planner_state_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      msg_autonomy.planner_state = msg->data;
    }

    void overtake_state_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
      msg_autonomy.overtake_state = msg->data;
    }

    void followd_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      curr_followd = msg->data;
    }

    void planner_controlvel_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      planner_controlvel = msg->data;
    }

    void opponent_dist_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      opponent_dist = msg->data;
    }

    void planner_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      planner_status = msg->data;
    }

    void planner_debug_state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      std::smatch m;
      if (regex_search(msg->data, m, maxspeed_pattern)) {
        planner_maxspeed = std::stof(m[1]);
      }
      if (regex_search(msg->data, m, plannerzone_pattern)) {
        planner_zone = zones[m[1]];
      }
      if (regex_search(msg->data, m, plannerstartzone_pattern)) {
        planner_startzone = (m[1].compare("True") == 0);
      }
    }

    void using_mixnet_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      using_mixnet = msg->data;
    }

    void debug1_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      debug1 = msg->data;
    }

    //void debug2_callback(const std_msgs::msg::Float32::SharedPtr msg)
    //{
    //  debug2 = msg->data;
    //}
    
    //void debug3_callback(const std_msgs::msg::Float32::SharedPtr msg)
    //{
    //  debug3 = msg->data;
    //}

    void mpc_debugging_callback(const deep_orange_msgs::msg::MpcTuning::SharedPtr msg)
    {
      if (msg->solution.state_cost.size() > 2) {
        mpc_tracking_cost = msg->solution.state_cost[0]+msg->solution.state_cost[1];
        mpc_heading_cost = msg->solution.state_cost[2];
      }
      if (msg->solution.input_cost.size() > 0) {
        mpc_input_cost = msg->solution.input_cost[0];
      }
      mpc_rho = msg->solution.rho;
      mpc_rhoQ = msg->solution.rhoq;
      if (msg->solution.q.size() > 2) {
        mpc_Q_track = msg->solution.q[0];
        mpc_Q_heading = msg->solution.q[2];
      }
      mpc_heading_error = msg->heading_error;
      mpc_solution_status = (msg->status.solver_status.compare("HpipmStatus::Success") == 0);
    }

    void long_errors_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      for (int i = 0; i < error_size; i++) {
        perception_errors[i] = msg->data[i];
      }
    }

    void lat_errors_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      for (int i = 0; i < error_size; i++) {
        perception_errors[error_size+i] = msg->data[i];
      }
    }

    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr receive_timer_;
    rclcpp::TimerBase::SharedPtr joystick_timer_;
    rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr sub_ct_report;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr sub_pt_report;
    rclcpp::Subscription<deep_orange_msgs::msg::MiscReport>::SharedPtr sub_misc_report_do;
    rclcpp::Subscription<deep_orange_msgs::msg::RcToCt>::SharedPtr sub_rc_to_ct;
    rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr sub_ws_report;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_lateral_error;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr sub_novatel;
    rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr sub_steering_cmd;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_mpc_hz;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_desvel;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_stop_reason;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_path;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_base_pose_rl;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_pose_rl;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vehicle_odom;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_clustering_hb;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_radar_hb;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_dnn_hb;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ekf_hb;
    rclcpp::Subscription<deep_orange_msgs::msg::TireReport>::SharedPtr sub_tire_report;
    rclcpp::Subscription<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr sub_brake_cmd;
    rclcpp::Subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr sub_accelerator_cmd;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_cmd;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ekf;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_radar;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ghost;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_clustering;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_dnn;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ref_traj;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_feasible_traj;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_opp_traj;
    rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_accel_filtered;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_lat_on;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_long_on;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_can_overtake;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_planner_action;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_planner_state;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_overtake_state;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_followd_feedback;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_planner_controlvel;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_opponent_dist;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_planner_status;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_planner_debug_state;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_using_mixnet;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_debug1;
    //rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_debug2;
    //rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_debug3;
    rclcpp::Subscription<deep_orange_msgs::msg::MpcTuning>::SharedPtr sub_mpc_debugging;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_p_long_errors;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_p_lat_errors;

    rclcpp::Publisher<deep_orange_msgs::msg::JoystickCommand>::SharedPtr pub_joystick_command; 
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_desired_velocity; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_switch_path; 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ct_input; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_lon; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_lat; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_mpc; 
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_followd; 
    rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr pub_spoof_flags; 

    deep_orange_msgs::msg::CtReport msg_ct_report;
    deep_orange_msgs::msg::PtReport msg_pt_report;
    deep_orange_msgs::msg::MiscReport msg_misc_report_do;
    deep_orange_msgs::msg::RcToCt msg_rc_to_ct;
    deep_orange_msgs::msg::Autonomy msg_autonomy;
    std_msgs::msg::Float32 msg_desvel;
    std_msgs::msg::String msg_reason;
    std_msgs::msg::String msg_path;
    raptor_dbw_msgs::msg::WheelSpeedReport msg_ws;
    geometry_msgs::msg::PointStamped msg_base_pose;
    geometry_msgs::msg::PointStamped msg_target_pose;
    nav_msgs::msg::Odometry msg_vehicle_pose;
    deep_orange_msgs::msg::TireReport msg_tire_report;
    sensor_msgs::msg::PointCloud2 msg_ref_traj;
    sensor_msgs::msg::PointCloud2 msg_feasible_traj;
    sensor_msgs::msg::PointCloud2 msg_opp_traj;
    nav_msgs::msg::Odometry msg_ekf;
    nav_msgs::msg::Odometry msg_radar;
    nav_msgs::msg::Odometry msg_ghost;
    nav_msgs::msg::Odometry msg_clustering;
    nav_msgs::msg::Odometry msg_dnn;
    geometry_msgs::msg::AccelWithCovarianceStamped msg_accel_filtered;
    bool lat_on = false;
    bool long_on = false;
    bool using_mixnet = false;
    float debug1 = 0.0;
    //float debug2 = 0.0;
    //float debug3 = 0.0;
    int steer_count = 0;
    float steer_hz = 0.0;
    float curr_followd = 80.0;
    float mpc_tracking_cost = 0.0;
    float mpc_heading_cost = 0.0;
    float mpc_input_cost = 0.0;
    float mpc_rho = 0.0;
    float mpc_rhoQ = 0.0;
    float mpc_Q_track = 0.0;
    float mpc_Q_heading = 0.0;
    float mpc_heading_error = 0.0;
    float opponent_dist = 0.0;
    float planner_controlvel = 0.0;
    float planner_maxspeed = 0.0;
    int planner_zone = 0;
    bool planner_startzone = false;
    bool mpc_solution_status = false;
    bool planner_status = false;
    const static int error_size = 5;
    float perception_errors[2*error_size] = { 0.0 };
    std::regex maxspeed_pattern = std::regex("Max Speed: ([\\d\\.]+),");
    std::regex plannerzone_pattern = std::regex("Zone: (\\w+),");
    std::regex plannerstartzone_pattern = std::regex("Starting Point: (\\w+),");

    //const google::protobuf::EnumDescriptor *detection_descriptor = Detection_DetectionId_descriptor();
    //const google::protobuf::EnumDescriptor *trajectory_descriptor = Trajectory_TrajectoryId_descriptor();
    UdpMessage udp_message;

    deep_orange_msgs::msg::JoystickCommand msg_joystick_command; 
    deep_orange_msgs::msg::RcToCt msg_spoof_flags; 
    double single_message_timeout = 1.0;
    rclcpp::Time startup_time = rclcpp::Clock().now() - rclcpp::Duration(5,0); // Subtract 5 seconds to prevent stale perception detections
    rclcpp::Time desired_velocity_time = startup_time;
    rclcpp::Time switch_path_time = startup_time;
    rclcpp::Time ct_input_time = startup_time;
    rclcpp::Time enable_lon_time = startup_time;
    rclcpp::Time enable_lat_time = startup_time;
    rclcpp::Time enable_mpc_time = startup_time;
    rclcpp::Time followd_time = startup_time;
    rclcpp::Time ekf_time = startup_time;
    rclcpp::Time radar_time = startup_time;
    rclcpp::Time ghost_time = startup_time;
    rclcpp::Time dnn_time = startup_time;
    rclcpp::Time clustering_time = startup_time;
    rclcpp::Time radar_hb_time = startup_time;
    rclcpp::Time clustering_hb_time = startup_time;
    rclcpp::Time dnn_hb_time = startup_time;
    rclcpp::Time ekf_hb_time = startup_time;
    rclcpp::Time steer_time = startup_time;
    rclcpp::Time feas_traj_time = startup_time;
    rclcpp::Time opp_traj_time = startup_time;

    //String to int conversions
    std::unordered_map<std::string, int> stops;
    std::unordered_map<std::string, int> paths = {{"",0},{"pits",1},{"raceline",2},{"alt",3},{"optimal",4},{"pit_in",5},{"graph",6}};
    std::unordered_map<std::string, int> actions = {{"",0},{"straight",1},{"follow",2},{"right",3},{"left",4}};
    //std::unordered_map<std::string, int> states = {{"",0},{"DEFENDER",1},{"CATCHUP",2},{"FOLLOW",3},{"OVERTAKE",4},{"NONE",5}};
    std::unordered_map<std::string, int> zones = {{"",0},{"TURN3",1},{"TURN4",2},{"FRONT",3},{"TURN1",4},{"TURN2",5},{"BACK",6}};

    // udp stuff
    boost::asio::io_service io_service_main;
    boost::asio::ip::udp::socket basestation_socket;
    boost::asio::ip::udp::socket racecar_socket;
    boost::asio::ip::udp::endpoint basestation_endpoint;
    boost::asio::ip::udp::endpoint racecar_endpoint;
    std::string basestation_ip;
    std::uint16_t basestation_port;
    std::string racecar_ip;
    std::uint16_t racecar_port;
    bool joystick_enable;
    float steering_ratio;
    int joystick_counter = 0;
    uint8_t read_buffer[2048];
    std::mutex recv_mutex;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry>());
  rclcpp::shutdown();
  return 0;
}

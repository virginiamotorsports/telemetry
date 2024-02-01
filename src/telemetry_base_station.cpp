#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <regex>
#include <fstream>

// ROS2 messages
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "deep_orange_msgs/msg/ct_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"
#include "deep_orange_msgs/msg/misc_report.hpp"
#include "deep_orange_msgs/msg/rc_to_ct.hpp"
#include "deep_orange_msgs/msg/autonomy.hpp"
#include "deep_orange_msgs/msg/tire_report_small.hpp"
#include "deep_orange_msgs/msg/joystick_command.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "uva_iac_msgs/msg/bounding_box_cav.hpp"
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

      // Get Parameters 
      //Pub topics:
      std::string ct_report_topic_, pt_report_topic_, misc_report_do_topic_, vehicle_viz_topic_, target_pose_topic_, base_pose_topic_, autonomy_topic_, tire_topic_, ekf_topic_, radar_topic_, planner_topic_, dnn_topic_, clustering_topic_, path_topic_, feasible_topic_, opp_path_topic_, accel_topic_, planner_maxspeed_topic_, planner_zone_topic_, planner_startzone_topic_, planner_controlvel_topic_, planner_status_topic_, opponent_distance_topic_, mpc_tracking_cost_topic_, mpc_heading_cost_topic_, mpc_input_cost_topic_, mpc_rho_topic_, mpc_rhoQ_topic_, mpc_Q_track_topic_, mpc_Q_heading_topic_, mpc_heading_error_topic_, mpc_solution_status_topic_, trajectory_error_topic_;

      this->declare_parameter<std::string>("ct_report_topic",           "/telemetry/ct_report");
      this->get_parameter(                 "ct_report_topic",           ct_report_topic_);
      this->declare_parameter<std::string>("pt_report_topic",           "/telemetry/pt_report");
      this->get_parameter(                 "pt_report_topic",           pt_report_topic_);
      this->declare_parameter<std::string>("misc_report_do_topic",      "/telemetry/misc_report_do");
      this->get_parameter(                 "misc_report_do_topic",      misc_report_do_topic_);
      this->declare_parameter<std::string>("vehicle_viz_topic",         "/telemetry/vehicle_viz");
      this->get_parameter(                 "vehicle_viz_topic",         vehicle_viz_topic_);
      this->declare_parameter<std::string>("target_pose_topic",         "/telemetry/target_pose_on_raceline");
      this->get_parameter(                 "target_pose_topic",         target_pose_topic_);
      this->declare_parameter<std::string>("base_pose_topic",           "/telemetry/base_pose_on_rl");
      this->get_parameter(                 "base_pose_topic",           base_pose_topic_);
      this->declare_parameter<std::string>("autonomy_topic",            "/telemetry/autonomy");
      this->get_parameter(                 "autonomy_topic",            autonomy_topic_);
      this->declare_parameter<std::string>("tire_topic",                "/telemetry/tire_report");
      this->get_parameter(                 "tire_topic",                tire_topic_);
      this->declare_parameter<std::string>("ekf_topic",                 "/telemetry/ekf_viz");
      this->get_parameter(                 "ekf_topic",                 ekf_topic_);
      this->declare_parameter<std::string>("radar_topic",               "/telemetry/radar_viz");
      this->get_parameter(                 "radar_topic",               radar_topic_);
      this->declare_parameter<std::string>("planner_topic",             "/telemetry/planner_viz");
      this->get_parameter(                 "planner_topic",             planner_topic_);
      this->declare_parameter<std::string>("dnn_topic",                 "/telemetry/dnn_viz");
      this->get_parameter(                 "dnn_topic",                 dnn_topic_);
      this->declare_parameter<std::string>("clustering_topic",          "/telemetry/clustering_viz");
      this->get_parameter(                 "clustering_topic",          clustering_topic_);
      this->declare_parameter<std::string>("path_topic",                "/telemetry/reference_trajectory");
      this->get_parameter(                 "path_topic",                path_topic_);
      this->declare_parameter<std::string>("feasible_topic",            "/telemetry/feasible_trajectory");
      this->get_parameter(                 "feasible_topic",            feasible_topic_);
      this->declare_parameter<std::string>("opp_path_topic",            "/telemetry/opp_trajectory");
      this->get_parameter(                 "opp_path_topic",            opp_path_topic_);
      this->declare_parameter<std::string>("accel_topic",               "/telemetry/vehicle_accel");
      this->get_parameter(                 "accel_topic",               accel_topic_);
      this->declare_parameter<std::string>("planner_maxspeed_topic",    "/telemetry/planner_maxspeed");
      this->get_parameter(                 "planner_maxspeed_topic",    planner_maxspeed_topic_);
      this->declare_parameter<std::string>("planner_zone_topic",        "/telemetry/planner_zone");
      this->get_parameter(                 "planner_zone_topic",        planner_zone_topic_);
      this->declare_parameter<std::string>("planner_startzone_topic",   "/telemetry/planner_startzone");
      this->get_parameter(                 "planner_startzone_topic",   planner_startzone_topic_);
      this->declare_parameter<std::string>("planner_controlvel_topic",  "/telemetry/planner_controlvel");
      this->get_parameter(                 "planner_controlvel_topic",  planner_controlvel_topic_);
      this->declare_parameter<std::string>("planner_status_topic",      "/telemetry/planner_status");
      this->get_parameter(                 "planner_status_topic",      planner_status_topic_);
      this->declare_parameter<std::string>("opponent_distance_topic",   "/telemetry/opponent_distance");
      this->get_parameter(                 "opponent_distance_topic",   opponent_distance_topic_);
      this->declare_parameter<std::string>("mpc_tracking_cost_topic",   "/mpc/tracking_cost");
      this->get_parameter(                 "mpc_tracking_cost_topic",   mpc_tracking_cost_topic_);
      this->declare_parameter<std::string>("mpc_heading_cost_topic",    "/mpc/heading_cost");
      this->get_parameter(                 "mpc_heading_cost_topic",    mpc_heading_cost_topic_);
      this->declare_parameter<std::string>("mpc_input_cost_topic",      "/mpc/input_cost");
      this->get_parameter(                 "mpc_input_cost_topic",      mpc_input_cost_topic_);
      this->declare_parameter<std::string>("mpc_rho_topic",             "/mpc/rho");
      this->get_parameter(                 "mpc_rho_topic",             mpc_rho_topic_);
      this->declare_parameter<std::string>("mpc_rhoQ_topic",            "/mpc/rhoQ");
      this->get_parameter(                 "mpc_rhoQ_topic",            mpc_rhoQ_topic_);
      this->declare_parameter<std::string>("mpc_Q_track_topic",         "/mpc/Q_track");
      this->get_parameter(                 "mpc_Q_track_topic",         mpc_Q_track_topic_);
      this->declare_parameter<std::string>("mpc_Q_heading",             "/mpc/Q_heading");
      this->get_parameter(                 "mpc_Q_heading",             mpc_Q_heading_topic_);
      this->declare_parameter<std::string>("mpc_heading_error_topic",   "/mpc/heading_error");
      this->get_parameter(                 "mpc_heading_error_topic",   mpc_heading_error_topic_);
      this->declare_parameter<std::string>("mpc_solution_status_topic", "/mpc/solution_status");
      this->get_parameter(                 "mpc_solution_status_topic", mpc_solution_status_topic_);
      this->declare_parameter<std::string>("trajectory_error_topic",    "/telemetry/trajectory_errors");
      this->get_parameter(                 "trajectory_error_topic",    trajectory_error_topic_);

      //Sub topics:
      std::string joy_accel_topic, joy_brake_topic, joy_steer_topic, joy_gear_topic, heartbeat_topic, e_stop_topic;
      this->declare_parameter<std::string>("basestation_ip", "192.168.0.201");
      this->get_parameter("basestation_ip", basestation_ip);

      this->declare_parameter<std::string>("racecar_ip", "192.168.0.200");
      this->get_parameter<std::string>("racecar_ip", racecar_ip);

      this->declare_parameter<std::uint16_t>("basestation_port", 23431);
      this->get_parameter<std::uint16_t>("basestation_port", basestation_port);

      this->declare_parameter<std::uint16_t>("racecar_port", 23531);
      this->get_parameter<std::uint16_t>("racecar_port", racecar_port);

      // ROS2 topics which should be republished
      pub_ct_report = this->create_publisher<deep_orange_msgs::msg::CtReport>(ct_report_topic_, 1);
      pub_pt_report = this->create_publisher<deep_orange_msgs::msg::PtReport>(pt_report_topic_, 1);
      pub_misc_report_do = this->create_publisher<deep_orange_msgs::msg::MiscReport>(misc_report_do_topic_, 1);
      pub_vehicle_viz = this->create_publisher<geometry_msgs::msg::PoseStamped>(vehicle_viz_topic_, 1);
      pub_target_pose_rl = this->create_publisher<geometry_msgs::msg::PointStamped>(target_pose_topic_, 1);
      pub_base_pose_rl = this->create_publisher<geometry_msgs::msg::PointStamped>(base_pose_topic_, 1);
      pub_autonomy = this->create_publisher<deep_orange_msgs::msg::Autonomy>(autonomy_topic_, 1);
      pub_tire_report = this->create_publisher<deep_orange_msgs::msg::TireReportSmall>(tire_topic_, 1);
      pub_ekf = this->create_publisher<uva_iac_msgs::msg::BoundingBoxCav>(ekf_topic_, 1);
      pub_radar = this->create_publisher<uva_iac_msgs::msg::BoundingBoxCav>(radar_topic_, 1);
      pub_ghost = this->create_publisher<uva_iac_msgs::msg::BoundingBoxCav>(planner_topic_, 1);
      pub_dnn = this->create_publisher<uva_iac_msgs::msg::BoundingBoxCav>(dnn_topic_, 1);
      pub_clustering = this->create_publisher<uva_iac_msgs::msg::BoundingBoxCav>(clustering_topic_, 1);
      pub_ref_traj = this->create_publisher<sensor_msgs::msg::PointCloud2>(path_topic_, 1);
      pub_feas_traj = this->create_publisher<sensor_msgs::msg::PointCloud2>(feasible_topic_, 1);
      pub_opp_traj = this->create_publisher<sensor_msgs::msg::PointCloud2>(opp_path_topic_, 1);
      pub_accel = this->create_publisher<geometry_msgs::msg::PointStamped>(accel_topic_, 1);
      pub_planner_maxspeed = this->create_publisher<std_msgs::msg::Float32>(planner_maxspeed_topic_, 1);
      pub_planner_zone = this->create_publisher<std_msgs::msg::Int32>(planner_zone_topic_, 1);
      pub_planner_startzone = this->create_publisher<std_msgs::msg::Bool>(planner_startzone_topic_, 1);
      pub_planner_controlvel = this->create_publisher<std_msgs::msg::Float32>(planner_controlvel_topic_, 1);
      pub_planner_status = this->create_publisher<std_msgs::msg::Bool>(planner_status_topic_, 1);
      pub_opponent_distance = this->create_publisher<std_msgs::msg::Float32>(opponent_distance_topic_, 1);
      pub_debug1 = this->create_publisher<std_msgs::msg::Float32>("/telemetry/debug1", 1);
      pub_debug2 = this->create_publisher<std_msgs::msg::Float32>("/telemetry/debug2", 1);
      pub_debug3 = this->create_publisher<std_msgs::msg::Float32>("/telemetry/debug3", 1);
      pub_using_mixnet = this->create_publisher<std_msgs::msg::Bool>("/telemetry/using_mixnet", 1);
      pub_mpc_rate = this->create_publisher<std_msgs::msg::Float32>("/telemetry/mpc_rate", 1);
      pub_feas_traj_fresh = this->create_publisher<std_msgs::msg::Bool>("/telemetry/feas_traj_fresh", 1);
      pub_opp_traj_fresh = this->create_publisher<std_msgs::msg::Bool>("/telemetry/opp_traj_fresh", 1);
      // MPC Debug Topics
      pub_mpc_tracking_cost = this->create_publisher<std_msgs::msg::Float32>(mpc_tracking_cost_topic_, 1);
      pub_mpc_heading_cost = this->create_publisher<std_msgs::msg::Float32>(mpc_heading_cost_topic_, 1);
      pub_mpc_input_cost = this->create_publisher<std_msgs::msg::Float32>(mpc_input_cost_topic_, 1);
      pub_mpc_rho = this->create_publisher<std_msgs::msg::Float32>(mpc_rho_topic_, 1);
      pub_mpc_rhoQ = this->create_publisher<std_msgs::msg::Float32>(mpc_rhoQ_topic_, 1);
      pub_mpc_Q_track = this->create_publisher<std_msgs::msg::Float32>(mpc_Q_track_topic_, 1);
      pub_mpc_Q_heading = this->create_publisher<std_msgs::msg::Float32>(mpc_Q_heading_topic_, 1);
      pub_mpc_heading_error = this->create_publisher<std_msgs::msg::Float32>(mpc_heading_error_topic_, 1);
      pub_mpc_solution_status = this->create_publisher<std_msgs::msg::Bool>(mpc_solution_status_topic_, 1);

      pub_trajectory_errors = this->create_publisher<std_msgs::msg::Float32MultiArray>(trajectory_error_topic_, 1);

      // ROS2 Topics which should be subscribed
      m_joystick_command = this->create_subscription<deep_orange_msgs::msg::JoystickCommand>("/joystick/command", qos, std::bind(&Telemetry::joystick_callback, this, std::placeholders::_1));
      m_desired_velocity = this->create_subscription<std_msgs::msg::Float32>("/vehicle/set_desired_velocity", 1, std::bind(&Telemetry::desired_velocity_callback, this, std::placeholders::_1));
      m_switch_path = this->create_subscription<std_msgs::msg::String>("/switch_path", 1, std::bind(&Telemetry::switch_path_callback, this, std::placeholders::_1));
      m_ct_input = this->create_subscription<std_msgs::msg::Int32>("/ct_input", 1, std::bind(&Telemetry::ct_input_callback, this, std::placeholders::_1));
      m_enable_lon = this->create_subscription<std_msgs::msg::Bool>("/enable_long", 1, std::bind(&Telemetry::enable_lon_callback, this, std::placeholders::_1));
      m_enable_lat = this->create_subscription<std_msgs::msg::Bool>("/enable_lat", 1, std::bind(&Telemetry::enable_lat_callback, this, std::placeholders::_1));
      m_enable_mpc = this->create_subscription<std_msgs::msg::Bool>("/enable_mpc", 1, std::bind(&Telemetry::enable_mpc_callback, this, std::placeholders::_1));
      m_followd = this->create_subscription<std_msgs::msg::Bool>("/followd", 1, std::bind(&Telemetry::followd_callback, this, std::placeholders::_1));
      m_track_flag = this->create_subscription<std_msgs::msg::Int32>("/track_flag", 1, std::bind(&Telemetry::track_flag_callback, this, std::placeholders::_1));
      m_veh_flag = this->create_subscription<std_msgs::msg::Int32>("/veh_flag", 1, std::bind(&Telemetry::veh_flag_callback, this, std::placeholders::_1));
      m_def_speed = this->create_subscription<std_msgs::msg::Int32>("/defender_speed", 1, std::bind(&Telemetry::def_speed_callback, this, std::placeholders::_1));
      m_debug_sub = this->create_subscription<std_msgs::msg::Bool>("/debug", 1, std::bind(&Telemetry::debug_callback, this, std::placeholders::_1));

      // timer which handles sending packets
      send_timer_ = this->create_wall_timer( std::chrono::milliseconds(10), std::bind(&Telemetry::send_callback, this) );
      // timer which handles receiving packets
      receive_timer_ = this->create_wall_timer( std::chrono::milliseconds(50), std::bind(&Telemetry::receive_callback, this) );

      // setup multiarray message
      for (int i = 0; i < error_size*2; i++) {
        msg_trajectory_errors.data.push_back(0.0);
      }

      // setup UDP interfaces
      racecar_socket.open(boost::asio::ip::udp::v4());
      racecar_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(racecar_ip), racecar_port);
      RCLCPP_INFO(this->get_logger(), "Established connection to send to remote at : %s:%u", racecar_ip.c_str(), racecar_port);

      basestation_socket.open(boost::asio::ip::udp::v4());
      basestation_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(basestation_ip), basestation_port);
      basestation_socket.bind(basestation_endpoint);
      basestation_socket.non_blocking(true);
      RCLCPP_INFO(this->get_logger(), "Established connection to receive from remote at : %s:%u", basestation_ip.c_str(), basestation_port);

      // Build int-to-string conversion map
      stops.push_back("");
      build_vector(&stops, ament_index_cpp::get_package_share_directory("node_health_monitor")+"/src/node_health_monitor.cpp", "reason.data = \\\"(.+)\\\"");
      build_vector(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "pub_warning\\(\\\"(.+)\\\"\\)");
      build_vector(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "hstop_car\\(\\\"(.+)\\\"\\)");
      build_vector(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "gstop_car\\(\\\"(.+)\\\"\\)");
      build_vector(&stops, ament_index_cpp::get_package_share_directory("gracefulstop")+"/src/graceful_stop_node.cpp", "estop_car\\(\\\"(.+)\\\"\\)");
    }

  private:
    void send_callback()
    {   
      BaseToCar bst_message;
      bst_message.set_counter(msg_joystick_cmd.counter);
      bst_message.set_emergency_stop(msg_joystick_cmd.emergency_stop);
      bst_message.set_joy_enable(msg_joystick_cmd.joy_enable);
      bst_message.set_steering_cmd(msg_joystick_cmd.steering_cmd);
      bst_message.set_brake_cmd(msg_joystick_cmd.brake_cmd);
      bst_message.set_accelerator_cmd(msg_joystick_cmd.accelerator_cmd);
      bst_message.set_gear_cmd(msg_joystick_cmd.gear_cmd);
      bst_message.set_desired_velocity(desired_velocity);
      bst_message.set_switch_path(switch_path);
      bst_message.set_ct_input(ct_input);
      bst_message.set_enable_lon(enable_lon);
      bst_message.set_enable_lat(enable_lat);
      bst_message.set_enable_mpc(enable_mpc);
      bst_message.set_follow_dist(inc_followd);
      bst_message.set_track_flag(track_flag);
      bst_message.set_veh_flag(veh_flag);
      bst_message.set_def_speed(def_speed);

      if (--desired_velocity_reset <= 0) {
        desired_velocity = -1.0;
        desired_velocity_reset = 0;
      }
      if (--switch_path_reset <= 0) {
        switch_path = "";
        switch_path_reset = 0;
      }
      if (--ct_input_reset <= 0) {
        ct_input = -1;
        ct_input_reset = 0;
      }
      if (--enable_lon_reset <= 0) {
        enable_lon = -1;
        enable_lon_reset = 0;
      }
      if (--enable_lat_reset <= 0) {
        enable_lat = -1;
        enable_lat_reset = 0;
      }
      if (--enable_mpc_reset <= 0) {
        enable_mpc = -1;
        enable_mpc_reset = 0;
      }
      if (--inc_followd_reset <= 0) {
        inc_followd = -1;
        inc_followd_reset = 0;
      }

      udp_message.clear_udp_data();
      udp_message.mutable_bst_data()->CopyFrom(bst_message);
      send_message(&udp_message); // Send Basestation Message
    }

    void receive_callback()
    {
      boost::system::error_code err;
      std::fill_n(read_buffer,2048,0);
      // read all messages from udp socket
      while (basestation_socket.receive_from(boost::asio::buffer(read_buffer), basestation_endpoint, 0, err) != 0) {
        recv_mutex.lock();
        google::protobuf::io::CodedInputStream coded_input_stream(read_buffer, 2048);
        uint32_t size = 0;
        if (!coded_input_stream.ReadVarint32(&size)) {
          return;
        }

        UdpMessage recv_telemetry_buffer;
        recv_telemetry_buffer.ParseFromCodedStream(&coded_input_stream);
        recv_mutex.unlock();

        if (debug && recv_telemetry_buffer.has_vehicle_data()) {
          RCLCPP_INFO(this->get_logger(), "Writing buffer to file");
          std::ofstream MyFile("telem_debug.txt");
          for (long unsigned int j = 0; j < sizeof(read_buffer); j++) {
            MyFile << read_buffer[j];
          }
          MyFile.close();

          char* coded_debug = new char[size + 1];
          coded_input_stream.ReadRaw(coded_debug, size);
          coded_debug[size] = '\0';
          std::ofstream MyFile2("telem_debug2.txt");
          for (long unsigned int j = 0; j < size; j++) {
            MyFile2 << coded_debug[j];
          }
          MyFile2.close();
          debug = false;
        }

        if (recv_telemetry_buffer.has_vehicle_data()) {
          VehicleData* data = recv_telemetry_buffer.mutable_vehicle_data();

          msg_ct_report.track_flag_ack = data->track_flag_ack();
          msg_ct_report.veh_flag_ack = data->veh_flag_ack();
          msg_ct_report.ct_state = data->ct_state();

          msg_pt_report.fuel_pressure = data->fuel_pressure();
          msg_pt_report.transmission_oil_temperature = data->transmission_oil_temperature();
          msg_pt_report.engine_oil_temperature = data->engine_oil_temperature();
          msg_pt_report.engine_coolant_temperature = data->engine_coolant_temperature();
          msg_pt_report.engine_rpm = data->engine_rpm();
          msg_pt_report.current_gear = data->current_gear();

          msg_misc_report_do.battery_voltage = data->battery_voltage();
          msg_misc_report_do.sys_state = data->sys_state();

          msg_autonomy.lateral_error = data->lateral_error();
          msg_autonomy.lat_stdev = data->lat_stdev();
          msg_autonomy.lon_stdev = data->lon_stdev();
          msg_autonomy.steering_cmd = data->steering_cmd();
          msg_autonomy.desired_velocity_readout = data->desired_velocity_readout();
          msg_autonomy.current_velocity = data->current_velocity();
          msg_autonomy.radar_died = radar_died;
          msg_autonomy.clustering_died = clustering_died;
          msg_autonomy.dnn_died = dnn_died;
          msg_autonomy.ekf_died = ekf_died;
          msg_autonomy.target_speed = data->target_speed();
          msg_autonomy.brake_cmd = data->brake_cmd();
          msg_autonomy.accelerator_cmd = data->accelerator_cmd();
          msg_autonomy.gear_cmd = data->gear_cmd();
          msg_autonomy.lat_on = data->lat_on();
          msg_autonomy.long_on = data->long_on();
          msg_autonomy.can_overtake = data->can_overtake();

          if (data->stop_reason() < stops.size()){
            msg_autonomy.stop_reason = stops[data->stop_reason()];
          } else {
            msg_autonomy.stop_reason = "Unknown";
          }
          if (data->path_name() < paths.size()){
            msg_autonomy.path_name = paths[data->path_name()];
          } else {
            msg_autonomy.path_name = "Unknown";
          }
          if (data->planner_action() < actions.size()){
            msg_autonomy.planner_output = actions[data->planner_action()];
          } else {
            msg_autonomy.planner_output = "Unknown";
          }
          /*if (data->planner_state() < states.size()) {
            msg_autonomy.planner_state = states[data->planner_state()];
          } else {
            msg_autonomy.planner_state = "Unknown";
          }*/
          msg_autonomy.planner_state = data->planner_state();
          msg_autonomy.overtake_state = data->overtake_state();

          msg_vehicle_viz.pose.position.x = data->veh_x();
          msg_vehicle_viz.pose.position.y = data->veh_y();
          msg_vehicle_viz.pose.orientation.x = data->veh_ang();

          msg_target_pose.point.x = data->target_point_x();
          msg_target_pose.point.y = data->target_point_y();
          msg_base_pose.point.x = data->base_pose_x();
          msg_base_pose.point.y = data->base_pose_y();

          msg_accel.point.x = data->accel_x();
          msg_accel.point.y = data->accel_y();

          msg_planner_maxspeed.data = data->planner_maxspeed();
          msg_planner_zone.data = data->planner_zone();
          msg_planner_startzone.data = data->planner_startzone();
          msg_planner_controlvel.data = data->planner_controlvel();
          msg_planner_status.data = data->planner_status();
          msg_opponent_distance.data = data->opponent_dist();

          msg_tire_report.fl_tire_temperature = data->fl_tire_temperature();
          msg_tire_report.fr_tire_temperature = data->fr_tire_temperature();
          msg_tire_report.rl_tire_temperature = data->rl_tire_temperature();
          msg_tire_report.rr_tire_temperature = data->rr_tire_temperature();

          float debug1 = data->debug1();
          float debug2 = data->debug2();
          float debug3 = data->debug3();

          for (int i = 0; i < data->perception_history_size(); i++) {
            msg_trajectory_errors.data[i] = data->perception_history(i);
          }

          std_msgs::msg::Float32 msg_debug1 = std_msgs::msg::Float32();
          std_msgs::msg::Float32 msg_debug2 = std_msgs::msg::Float32();
          std_msgs::msg::Float32 msg_debug3 = std_msgs::msg::Float32();
          msg_debug1.data = debug1;
          msg_debug2.data = debug2;
          msg_debug3.data = debug3;

          msg_using_mixnet.data = data->using_mixnet();
          msg_mpc_rate.data = data->mpc_rate();
          msg_mpc_tracking_cost.data = data->mpc_tracking_cost();
          msg_mpc_heading_cost.data = data->mpc_heading_cost();
          msg_mpc_input_cost.data = data->mpc_input_cost();
          msg_mpc_rho.data = data->mpc_rho();
          msg_mpc_rhoQ.data = data->mpc_rhoq();
          msg_mpc_Q_track.data = data->mpc_q_track();
          msg_mpc_Q_heading.data = data->mpc_q_heading();
          msg_mpc_heading_error.data = data->mpc_heading_error();
          msg_mpc_solution_status.data = data->mpc_solution_status();

          pub_ct_report->publish(msg_ct_report);
          pub_pt_report->publish(msg_pt_report);
          pub_misc_report_do->publish(msg_misc_report_do);
          pub_autonomy->publish(msg_autonomy);
          pub_vehicle_viz->publish(msg_vehicle_viz);
          pub_target_pose_rl->publish(msg_target_pose);
          pub_base_pose_rl->publish(msg_base_pose);
          pub_accel->publish(msg_accel);
          pub_planner_maxspeed->publish(msg_planner_maxspeed);
          pub_planner_zone->publish(msg_planner_zone);
          pub_planner_startzone->publish(msg_planner_startzone);
          pub_planner_controlvel->publish(msg_planner_controlvel);
          pub_planner_status->publish(msg_planner_status);
          pub_opponent_distance->publish(msg_opponent_distance);
          pub_tire_report->publish(msg_tire_report);
          pub_debug1->publish(msg_debug1);
          pub_debug2->publish(msg_debug2);
          pub_debug3->publish(msg_debug3);
          pub_using_mixnet->publish(msg_using_mixnet);
          pub_mpc_rate->publish(msg_mpc_rate);
          pub_mpc_tracking_cost->publish(msg_mpc_tracking_cost);
          pub_mpc_heading_cost->publish(msg_mpc_heading_cost);
          pub_mpc_input_cost->publish(msg_mpc_input_cost);
          pub_mpc_rho->publish(msg_mpc_rho);
          pub_mpc_rhoQ->publish(msg_mpc_rhoQ);
          pub_mpc_Q_track->publish(msg_mpc_Q_track);
          pub_mpc_Q_heading->publish(msg_mpc_Q_heading);
          pub_mpc_solution_status->publish(msg_mpc_solution_status);
          pub_mpc_heading_error->publish(msg_mpc_heading_error);
          pub_trajectory_errors->publish(msg_trajectory_errors);
        }
        else if (recv_telemetry_buffer.has_perception_data()) {
          PerceptionData* data = recv_telemetry_buffer.mutable_perception_data();

          for (int i = 0; i < data->detections_size(); i++) {
            msg_detection = uva_iac_msgs::msg::BoundingBoxCav();
            msg_detection.center.position.x = data->detections(i).center_x();
            msg_detection.center.position.y = data->detections(i).center_y();
            msg_detection.center.orientation.x = data->detections(i).angle();
            if (data->detections(i).fresh()) {
              msg_detection.center.orientation.y = 1;
            } else {
              msg_detection.center.orientation.y = 0;
            }
            msg_detection.velocity = data->detections(i).velocity();

            // Publish detection on correct topic
            switch (data->detections(i).id()) {
              case Detection_DetectionId_RADAR:
                pub_radar->publish(msg_detection);
                radar_died = data->detections(i).source_died();
                break;
              case Detection_DetectionId_CLUSTERING:
                pub_clustering->publish(msg_detection);
                clustering_died = data->detections(i).source_died();
                break;
              case Detection_DetectionId_DNN:
                pub_dnn->publish(msg_detection);
                dnn_died = data->detections(i).source_died();
                break;
              case Detection_DetectionId_EKF:
                pub_ekf->publish(msg_detection);
                ekf_died = data->detections(i).source_died();
                break;
              case Detection_DetectionId_GHOST:
                pub_ghost->publish(msg_detection);
                break;
              case Detection_DetectionId_LIDAR:
              default:
                // Ignore unknown ids
                break;
            }
          }
        }
        else if (recv_telemetry_buffer.has_trajectory_data()) {
          TrajectoryData* data = recv_telemetry_buffer.mutable_trajectory_data();

          for (int i = 0; i < data->trajectories_size(); i++) {
            if (data->trajectories(i).path_x_size() > 0) {
              msg_traj = sensor_msgs::msg::PointCloud2();
              msg_traj.height = 1;
              msg_traj.width = data->trajectories(i).path_x_size();
              msg_traj.is_bigendian = false;
              msg_traj.is_dense = false;
              sensor_msgs::PointCloud2Modifier pcd_modifier(msg_traj);
              pcd_modifier.setPointCloud2Fields(2,
                  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                  "y", 1, sensor_msgs::msg::PointField::FLOAT32);
                  //"kappa", 1, sensor_msgs::msg::PointField::FLOAT32);
              sensor_msgs::PointCloud2Iterator<float> iter_x(msg_traj, "x");
              sensor_msgs::PointCloud2Iterator<float> iter_y(msg_traj, "y");
              for (int j = 0; j < data->trajectories(i).path_x_size(); j++, ++iter_x, ++iter_y) {
                *iter_x = data->trajectories(i).path_x(j);
                *iter_y = data->trajectories(i).path_y(j);
              }

              // Publish path on correct topic
              switch (data->trajectories(i).id()) {
                case Trajectory_TrajectoryId_REFERENCE:
                  pub_ref_traj->publish(msg_traj);
                  break;
                case Trajectory_TrajectoryId_FEASIBLE:
                  pub_feas_traj->publish(msg_traj);
                  msg_feas_traj_fresh.data = data->trajectories(i).fresh();
                  pub_feas_traj_fresh->publish(msg_feas_traj_fresh);
                  break;
                case Trajectory_TrajectoryId_OPPONENT:
                  pub_opp_traj->publish(msg_traj);
                  msg_opp_traj_fresh.data = data->trajectories(i).fresh();
                  pub_opp_traj_fresh->publish(msg_opp_traj_fresh);
                  break;
                default:
                  // Ignore unknown ids
                  break;
              }
            }
          }
        }
        std::fill_n(read_buffer,2048,0); // Clear read_buffer for the next message
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
      racecar_socket.send_to(output_stream_buffer.data(), racecar_endpoint, 0, err);
      return err;
    }

    void build_vector(std::vector<std::string> *vector, std::string fname, std::string pattern) {
      std::regex r_pattern(pattern);
      std::smatch m;
      std::string line;
      std::ifstream myfile(fname);
      if (myfile.is_open()) {
        while (getline(myfile,line)) {
          if (regex_search(line, m, r_pattern)) {
            vector->push_back(m[1]);
          }
        }
        myfile.close();
      }
    }

    void joystick_callback(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg)
    { 
        msg_joystick_cmd = *msg;
    }
    void desired_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    { 
        desired_velocity = msg->data;
        desired_velocity_reset = 5;
    }
    void switch_path_callback(const std_msgs::msg::String::SharedPtr msg)
    { 
        switch_path = msg->data;
        switch_path_reset = 5;
    }
    void ct_input_callback(const std_msgs::msg::Int32::SharedPtr msg)
    { 
        ct_input = msg->data;
        ct_input_reset = 5;
    }
    void enable_lon_callback(const std_msgs::msg::Bool::SharedPtr msg)
    { 
        if (msg->data) {
          enable_lon = 1;
        } else {
          enable_lon = 0;
        }
        enable_lon_reset = 5;
    }
    void enable_lat_callback(const std_msgs::msg::Bool::SharedPtr msg)
    { 
        if (msg->data) {
          enable_lat = 1;
        } else {
          enable_lat = 0;
        }
        enable_lat_reset = 5;
    }
    void enable_mpc_callback(const std_msgs::msg::Bool::SharedPtr msg)
    { 
        if (msg->data) {
          enable_mpc = 1;
        } else {
          enable_mpc = 0;
        }
        enable_mpc_reset = 5;
    }
    void followd_callback(const std_msgs::msg::Bool::SharedPtr msg)
    { 
        if (msg->data) {
          inc_followd = 1;
        } else {
          inc_followd = 0;
        }
        inc_followd_reset = 5;
    }
    void track_flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
    { 
      track_flag = msg->data;
    }
    void veh_flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
    { 
      veh_flag = msg->data;
    }
    void def_speed_callback(const std_msgs::msg::Int32::SharedPtr msg)
    { 
      def_speed = msg->data;
    }
    void debug_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      debug = msg->data;
    }

    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr receive_timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::CtReport>::SharedPtr pub_ct_report;
    rclcpp::Publisher<deep_orange_msgs::msg::PtReport>::SharedPtr pub_pt_report;
    rclcpp::Publisher<deep_orange_msgs::msg::MiscReport>::SharedPtr pub_misc_report_do;
    rclcpp::Publisher<deep_orange_msgs::msg::Autonomy>::SharedPtr pub_autonomy;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_base_pose_rl;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_pose_rl;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_vehicle_viz;
    rclcpp::Publisher<deep_orange_msgs::msg::TireReportSmall>::SharedPtr pub_tire_report;
    rclcpp::Publisher<uva_iac_msgs::msg::BoundingBoxCav>::SharedPtr pub_ekf;
    rclcpp::Publisher<uva_iac_msgs::msg::BoundingBoxCav>::SharedPtr pub_radar;
    rclcpp::Publisher<uva_iac_msgs::msg::BoundingBoxCav>::SharedPtr pub_ghost;
    rclcpp::Publisher<uva_iac_msgs::msg::BoundingBoxCav>::SharedPtr pub_dnn;
    rclcpp::Publisher<uva_iac_msgs::msg::BoundingBoxCav>::SharedPtr pub_clustering;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ref_traj;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_feas_traj;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_opp_traj;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_accel;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_planner_maxspeed;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_planner_zone;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_planner_startzone;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_planner_controlvel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_planner_status;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_opponent_distance;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_debug1;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_debug2;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_debug3;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_using_mixnet;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_rate;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_feas_traj_fresh;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_opp_traj_fresh;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_tracking_cost;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_heading_cost;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_input_cost;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_rho;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_rhoQ;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_Q_track;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_Q_heading;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mpc_heading_error;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_mpc_solution_status;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_trajectory_errors;

    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr m_joystick_command;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_desired_velocity;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_switch_path;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_ct_input;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_enable_lon;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_enable_lat;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_enable_mpc;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_followd;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_track_flag;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_veh_flag;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_def_speed;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_debug_sub;

    deep_orange_msgs::msg::JoystickCommand msg_joystick_cmd;
    float desired_velocity = -1.0;
    std::string switch_path = "";
    int ct_input = -1;
    int enable_lon = -1;
    int enable_lat = -1;
    int enable_mpc = -1;
    int inc_followd = -1;
    int track_flag = 1;
    int veh_flag = 1;
    int def_speed = 0;
    int desired_velocity_reset = 0;
    int switch_path_reset = 0;
    int ct_input_reset = 0;
    int enable_lon_reset = 0;
    int enable_lat_reset = 0;
    int enable_mpc_reset = 0;
    int inc_followd_reset = 0;
    const static int error_size = 5;
    bool debug = false;

    //String to int conversions
    std::vector<std::string> stops;
    std::vector<std::string> paths = {"","pits","raceline","alt","optimal","pit_in","graph"};
    std::vector<std::string> actions = {"","straight","follow","right","left"};
    //std::vector<std::string> states = {"","DEFENDER","CATCHUP","FOLLOW","OVERTAKE","NONE"};

    UdpMessage udp_message;

    deep_orange_msgs::msg::CtReport msg_ct_report;
    deep_orange_msgs::msg::PtReport msg_pt_report;
    deep_orange_msgs::msg::MiscReport msg_misc_report_do;
    deep_orange_msgs::msg::RcToCt msg_rc_to_ct;
    deep_orange_msgs::msg::Autonomy msg_autonomy;
    geometry_msgs::msg::PointStamped msg_base_pose;
    geometry_msgs::msg::PointStamped msg_target_pose;
    geometry_msgs::msg::PoseStamped msg_vehicle_viz;
    deep_orange_msgs::msg::TireReportSmall msg_tire_report;
    uva_iac_msgs::msg::BoundingBoxCav msg_detection;
    sensor_msgs::msg::PointCloud2 msg_traj;
    geometry_msgs::msg::PointStamped msg_accel;
    std_msgs::msg::Float32 msg_planner_maxspeed;
    std_msgs::msg::Int32 msg_planner_zone;
    std_msgs::msg::Bool msg_planner_startzone;
    std_msgs::msg::Float32 msg_planner_controlvel;
    std_msgs::msg::Bool msg_planner_status;
    std_msgs::msg::Float32 msg_opponent_distance;
    std_msgs::msg::Bool msg_using_mixnet;
    std_msgs::msg::Float32 msg_mpc_rate;
    std_msgs::msg::Bool msg_feas_traj_fresh;
    std_msgs::msg::Bool msg_opp_traj_fresh;
    std_msgs::msg::Float32MultiArray msg_trajectory_errors;
    // Perception Health
    bool radar_died;
    bool clustering_died;
    bool dnn_died;
    bool ekf_died;
    // MPC Debug Message
    std_msgs::msg::Float32 msg_mpc_tracking_cost;
    std_msgs::msg::Float32 msg_mpc_heading_cost;
    std_msgs::msg::Float32 msg_mpc_input_cost;
    std_msgs::msg::Float32 msg_mpc_rho;
    std_msgs::msg::Float32 msg_mpc_rhoQ;
    std_msgs::msg::Float32 msg_mpc_Q_track;
    std_msgs::msg::Float32 msg_mpc_Q_heading;
    std_msgs::msg::Float32 msg_mpc_heading_error;
    std_msgs::msg::Bool msg_mpc_solution_status;
    
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

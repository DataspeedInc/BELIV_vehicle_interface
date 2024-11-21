/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "beliv_veh_interface.hpp"
#include <iostream>
#include <fstream> 

namespace ds_dbw_can {
BelivVehInterface::BelivVehInterface()
  :rclcpp::Node("beliv_veh_interface")
{
  /* set up parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");   //Frame ID
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 50.0);
  wheel_base_ = declare_parameter("wheel_base", 3.08);
  steering_ratio_ = declare_parameter("steering_ratio", 16.2);

 
  /* subscriber */
  using std::placeholders::_1;
  using std::placeholders::_2;

  //from DbwNode
  sub_brake_ = create_subscription<ds_dbw_msgs::msg::BrakeReport>("/vehicle/brake/report", rclcpp::QoS{2}, 
    std::bind(&BelivVehInterface::callbackBrakeRpt, this, _1));
//sub_throttle_ = create_subscription<ds_dbw_msgs::msg::ThrottleReport>("/vehicle/throttle_report", 2);
  sub_steering_ = std::make_unique<message_filters::Subscriber<ds_dbw_msgs::msg::SteeringReport>>(
    this, "/vehicle/steering/report");
  sub_gear_ = std::make_unique<message_filters::Subscriber<ds_dbw_msgs::msg::GearReport>>(
    this, "/vehicle/gear/report");
  sub_turn_signal_= std::make_unique<message_filters::Subscriber<ds_dbw_msgs::msg::TurnSignalReport>>(
    this, "/vehicle/turn_signal/report");
/*  sub_wheel_speeds_ = create_subscription<ds_dbw_msgs::msg::WheelSpeedReport>("/vehicle/wheel_speed_report", 2);
  sub_wheel_positions_ = create_subscription<ds_dbw_msgs::msg::WheelPositionReport>("/vehicle/wheel_position_report", 2);
  sub_tire_pressure_ = create_subscription<ds_dbw_msgs::msg::TirePressureReport>("/vehicle/tire_pressure_report", 2);
  sub_fuel_level_ = create_subscription<ds_dbw_msgs::msg::FuelLevelReport>("/vehicle/fuel_level_report", 2);
  sub_surround_ = create_subscription<ds_dbw_msgs::msg::SurroundReport>("/vehicle/surround_report", 2);
  sub_sonar_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("/vehicle/sonar_cloud", 2);
  sub_brake_info_ = create_subscription<ds_dbw_msgs::msg::BrakeInfoReport>("/vehicle/brake_info_report", 2);
  sub_throttle_info_ = create_subscription<ds_dbw_msgs::msg::ThrottleInfoReport>("/vehicle/throttle_info_report", 2);
  sub_driver_assist_ = create_subscription<ds_dbw_msgs::msg::DriverAssistReport>("/vehicle/driver_assist_report", 2);
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("/vehicle/imu/data_raw", 10);
  sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>("/vehicle/gps/fix", 10);
  sub_gps_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>("/vehicle/gps/vel", 10);
  sub_gps_time_ = create_subscription<sensor_msgs::msg::TimeReference>("/vehicle/gps/time", 10);
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>("/vehicle/twist", 10);   
 */
  //from Autoware
  sub_control_cmd_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 1, std::bind(&BelivVehInterface::callbackControlCmd, this, _1));
  //sub_control_cmd_ = std::make_unique<message_filters::Subscriber<autoware_control_msgs::msg::Control>>(
    //this, "/control/command/control_cmd");
  //sub_gear_cmd_ = create_subscription<GearCommand>("input/gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_gear_cmd_ = *msg; });
  //sub_manual_gear_cmd_ = create_subscription<GearCommand>("input/manual_gear_command", QoS{1},[this](const GearCommand::SharedPtr msg) { current_manual_gear_cmd_ = *msg; });

  sub_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>("/control/command/turn_indicators_cmd", 1,std::bind(&BelivVehInterface::callbackTurnIndicatorsCmd, this, _1));
  // sub_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>("/control/command/hazard_lights_cmd", 1,std::bind(&BelivVehInterface::callbackHazardLightsCmd, this, _1));
 
  sub_emergency_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&BelivVehInterface::callbackEmergencyCmd, this, _1));
  control_mode_server_ = create_service<ControlModeCommand>(
    "input/control_mode_request", std::bind(&BelivVehInterface::onControlModeRequest, this, _1, _2));

  //from UlcNode
  sub_ulc_rpt_ = std::make_unique<message_filters::Subscriber<ds_dbw_msgs::msg::UlcReport>>(
    this, "/vehicle/ulc/report");

  //from DbwNode
  sub_enable_ = create_subscription<std_msgs::msg::Bool>("/vehicle/dbw_enabled", rclcpp::QoS(2).transient_local(),
    std::bind(&BelivVehInterface::recvDbwEnabled, this, _1));   

  //synchronizer
  beliv_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<BelivFeedbacksSyncPolicy>>(
    BelivFeedbacksSyncPolicy(10), *sub_steering_, *sub_gear_, *sub_turn_signal_, *sub_ulc_rpt_);
  
  beliv_feedbacks_sync_->registerCallback(std::bind(
    &BelivVehInterface::callbackInterface, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4));

  /* publisher */
  //to UlcNode
  pub_ulc_cmd_=create_publisher<ds_dbw_msgs::msg::UlcCmd>("/vehicle/ulc/cmd", 2);
  pub_steering_cmd_ = create_publisher<ds_dbw_msgs::msg::SteeringCmd>("/vehicle/steering/cmd", 2);
  pub_turn_signal_cmd_ = create_publisher<ds_dbw_msgs::msg::TurnSignalCmd>("/vehicle/turn_signal/cmd", 2);
  // pub_hazard_lights_cmd_ = create_publisher<ds_dbw_msgs::msg::MiscCmd>("/vehicle/hazard_lights_cmd", 2);
  // //to DbwNode
  // pub_turn_signal_cmd_ = create_publisher<ds_dbw_msgs::msg::MiscCmd>(
    
  // )

  //to Autoware
  pub_battery_status_ = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>(
    "/vehicle/status/battery_charge", rclcpp::QoS{1});
  pub_control_mode_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  pub_vehicle_twist_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  pub_steering_status_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  pub_gear_status_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  pub_turn_indicators_status_ =
    create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  pub_hazard_lights_status_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
/*   pub_actuation_status_ =
    create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1); */
  pub_steering_wheel_status_ =
    create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);
  pub_door_status_ =
    create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);


  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&BelivVehInterface::publishCommands, this));

  RCLCPP_INFO_ONCE(get_logger(), "BelivVehInterface is ready.");
}

void BelivVehInterface::callbackInterface(
  const ds_dbw_msgs::msg::SteeringReport::ConstSharedPtr steering_rpt,
  const ds_dbw_msgs::msg::GearReport::ConstSharedPtr gear_rpt,
  const ds_dbw_msgs::msg::TurnSignalReport::ConstSharedPtr turn_signal_rpt,
  const ds_dbw_msgs::msg::UlcReport::ConstSharedPtr ulc_rpt)
{

  const float charge = 100.0; // dummy value to publish as battery charge status

  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  is_dbw_rpt_received_ = true;
  sub_steering_ptr_ = steering_rpt;
  sub_gear_ptr_ = gear_rpt;
  sub_turn_signal_ptr_ = turn_signal_rpt;
  sub_ulc_rpt_ptr_ = ulc_rpt;

  const double current_velocity = sub_ulc_rpt_ptr_->vel_meas;  // current vehicle velocity [m/s]
  const double current_steer_wheel = sub_steering_ptr_->steering_wheel_angle;  // current vehicle steering wheel angle [rad]
  const double current_steer = current_steer_wheel / steering_ratio_;

    /* publish battery charge status */
  {
    tier4_vehicle_msgs::msg::BatteryStatus battery_status_msg;
    battery_status_msg.stamp = header.stamp;

    battery_status_msg.energy_level = charge;  // dummy value

    pub_battery_status_->publish(battery_status_msg);
  
  }

  /* publish steering wheel status */
  {
    SteeringWheelStatusStamped steering_wheel_status_msg;
    steering_wheel_status_msg.stamp = header.stamp;
    //std::cout << "current_steer_wheel: " << current_steer_wheel << "rad" << std::endl;

    steering_wheel_status_msg.data = current_steer_wheel;
    pub_steering_wheel_status_->publish(steering_wheel_status_msg);
  
  }

  /* publish current steering status */
  {
    autoware_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = header.stamp;
    steer_msg.steering_tire_angle = current_steer;
    pub_steering_status_->publish(steer_msg);
  }

  /*publish Velocity Report*/
  {
    autoware_vehicle_msgs::msg::VelocityReport twist;
    twist.header = header;
    twist.longitudinal_velocity = current_velocity;                                 // [m/s]
    twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s] yaw rate

    pub_vehicle_twist_->publish(twist);
/*     RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "longitudinal_velocity = %f, heading_rate = %f", twist.longitudinal_velocity,
      twist.heading_rate
    );  */
    }

  /* publish vehicle status control_mode */
  {
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = header.stamp;

    if (dbw_enable_) {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    } else {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    }
    pub_control_mode_->publish(control_mode_msg);
  }

  /* publish current shift */
  {
    autoware_vehicle_msgs::msg::GearReport gear_report_msg;
    gear_report_msg.stamp = header.stamp;
    const auto opt_gear_report = toAutowareShiftReport(*sub_gear_ptr_);
    if (opt_gear_report) {
      gear_report_msg.report = opt_gear_report;
      pub_gear_status_->publish(gear_report_msg);
    }
  }

  /*publish TurnIndicatorsReport*/
  {
    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
    turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(*sub_turn_signal_ptr_);
    pub_turn_indicators_status_->publish(turn_msg);

    autoware_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(*sub_turn_signal_ptr_);
    pub_hazard_lights_status_->publish(hazard_msg);
  }

}


int32_t BelivVehInterface::toAutowareShiftReport(
  const ds_dbw_msgs::msg::GearReport & gear_rpt)
{
  using autoware_vehicle_msgs::msg::GearReport;
  using ds_dbw_msgs::msg::Gear;

  if (gear_rpt.gear.value == ds_dbw_msgs::msg::Gear::PARK) {
    return static_cast<int32_t>(GearReport::PARK);
  }
  if (gear_rpt.gear.value == ds_dbw_msgs::msg::Gear::REVERSE){
    return static_cast<int32_t>(GearReport::REVERSE);
  }
  if (gear_rpt.gear.value == ds_dbw_msgs::msg::Gear::NEUTRAL){
    return static_cast<int32_t>(GearReport::NEUTRAL);
  }
  if (gear_rpt.gear.value == ds_dbw_msgs::msg::Gear::DRIVE){
    return static_cast<int32_t>(GearReport::DRIVE);
  }
  if (gear_rpt.gear.value==ds_dbw_msgs::msg::Gear::LOW){
    return static_cast<int32_t>(GearReport::LOW);
  }
  return static_cast<int32_t>(GearReport::NONE);
}


void BelivVehInterface::callbackControlCmd(
  const autoware_control_msgs::msg::Control& msg)
{
  control_command_received_time_ = this->now();
  ulc_cmd_.header.frame_id = base_frame_id_;
  ulc_cmd_.header.stamp = get_clock()->now();
  // Populate command fields
  ulc_cmd_.cmd_type = ds_dbw_msgs::msg::UlcCmd::CMD_VELOCITY;
  ulc_cmd_.enable = true;

  if (msg.longitudinal.velocity  < -0.2352) {
    steering_cmd_.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE;
  }
  else if (msg.longitudinal.velocity < 0.2352) {
    steering_cmd_.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_CURVATURE;
  }
  else {
    steering_cmd_.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE;
  }
  // if ( msg.longitudinal.velocity < -0.5) {
  //   ulc_cmd_.linear_velocity = msg.longitudinal.velocity;
  // } else if (msg.longitudinal.velocity < -0.001) {
  //   ulc_cmd_.linear_velocity = -0.5;
  // } else if (msg.longitudinal.velocity < 0.001) {
  //   ulc_cmd_.linear_velocity = 0.0;
  // } else if (msg.longitudinal.velocity < 0.5) {
  //   ulc_cmd_.linear_velocity = 0.5;
  // } else {
  //   ulc_cmd_.linear_velocity = msg.longitudinal.velocity;
  // }

  ulc_cmd_.cmd = msg.longitudinal.velocity;
  if (steering_cmd_.cmd_type == ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE) {
    steering_cmd_.cmd = sub_ulc_rpt_ptr_->vel_meas * tan(msg.lateral.steering_tire_angle)/wheel_base_;
  }
  else {
    steering_cmd_.cmd = std::atan(msg.lateral.steering_tire_angle/steering_ratio_);
  }
  // steering_cmd_.cmd = sub_ulc_rpt_ptr_->vel_meas * tan(msg.lateral.steering_tire_angle)/wheel_base_;
  // steering_cmd_.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE;
  steering_cmd_.enable = true;

  // Set other fields to default values
  ulc_cmd_.clear = false;
  ulc_cmd_.enable_shift = true;
  ulc_cmd_.enable_shift_park = true;
  ulc_cmd_.limit_accel = 0;
  ulc_cmd_.limit_decel = 3.0;
  ulc_cmd_.limit_jerk_throttle=0;
  ulc_cmd_.limit_jerk_brake=0;
  //printf("ulc_cmd.linear_vel=%f", ulc_cmd_.linear_velocity);
}

void BelivVehInterface::callbackTurnIndicatorsCmd(
  const TurnIndicatorsCommand& turning_indicators_cmd)
{
  // set flag for publishing turning indicators
  // publish_turn_indicators_ = true;

  turn_signal_cmd_received_time_ = this->now();
  turn_signal_cmd_.header.frame_id = base_frame_id_;
  turn_signal_cmd_.header.stamp = get_clock()->now();
  
  if (turning_indicators_cmd.command == TurnIndicatorsCommand::NO_COMMAND) {
    turn_signal_cmd_.cmd.value = ds_dbw_msgs::msg::TurnSignal::NONE;
  }
  else if (turning_indicators_cmd.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    turn_signal_cmd_.cmd.value = ds_dbw_msgs::msg::TurnSignal::RIGHT;
  }
  else if (turning_indicators_cmd.command == TurnIndicatorsCommand::ENABLE_LEFT) {
    turn_signal_cmd_.cmd.value = ds_dbw_msgs::msg::TurnSignal::LEFT;
  }
  else if (turning_indicators_cmd.command == TurnIndicatorsCommand::DISABLE) {
    turn_signal_cmd_.cmd.value = ds_dbw_msgs::msg::TurnSignal::NONE;
  }

  // Set other fields to default values
  // turn_signal_cmd_.parking_brake.value = ds_dbw_msgs::msg::PrkBrkCmd::NONE;
}

// void BelivVehInterface::callbackHazardLightsCmd(
//   const HazardLightsCommand& hazard_lights_cmd)
//   {
//     // set flag for publishing turning indicators
//     // publish_turn_indicators_ = false;

//     hazard_lights_misc_command_received_time_ = this->now();
//     hazard_lights_misc_cmd_.header.frame_id = base_frame_id_;
//     hazard_lights_misc_cmd_.header.stamp = get_clock()->now();

    
//     if (hazard_lights_cmd.command == HazardLightsCommand::ENABLE) {
//       publish_turn_indicators_ = false;
//       hazard_lights_misc_cmd_.turn_signal.value = ds_dbw_msgs::msg::TurnSignal::HAZARD;
//     }
//     else if (hazard_lights_cmd.command == HazardLightsCommand::DISABLE) {
//       publish_turn_indicators_ = true;
//       hazard_lights_misc_cmd_.turn_signal.value = ds_dbw_msgs::msg::TurnSignal::NONE;
//     }
//     else if (hazard_lights_cmd.command == HazardLightsCommand::NO_COMMAND) {
//       publish_turn_indicators_ = true;
//       hazard_lights_misc_cmd_.turn_signal.value = ds_dbw_msgs::msg::TurnSignal::NONE;
//     }

//     // Set other fields to default values
//     hazard_lights_misc_cmd_.door.header.stamp = hazard_lights_misc_cmd_.header.stamp;
//     hazard_lights_misc_cmd_.door.select = 0;
//     hazard_lights_misc_cmd_.door.action = 0;
//     hazard_lights_misc_cmd_.ft_drv_temp.value = 0;
//     hazard_lights_misc_cmd_.ft_psg_temp.value = 0;
//     hazard_lights_misc_cmd_.ft_fan_speed.value = 0;
//     hazard_lights_misc_cmd_.max_ac.cmd = 0;
//     hazard_lights_misc_cmd_.ac.cmd = 0;
//     hazard_lights_misc_cmd_.ft_hvac.cmd = 0;
//     hazard_lights_misc_cmd_.auto_md.cmd = 0;
//     hazard_lights_misc_cmd_.recirc.cmd = 0;
//     hazard_lights_misc_cmd_.sync.cmd = 0;
//     hazard_lights_misc_cmd_.r_defr.cmd = 0;
//     hazard_lights_misc_cmd_.f_defr.cmd = 0;
//     hazard_lights_misc_cmd_.vent_mode.value = 0;
//     hazard_lights_misc_cmd_.heated_steering_wheel.cmd = 0;
//     hazard_lights_misc_cmd_.fl_heated_seat.value = 0;
//     hazard_lights_misc_cmd_.fr_heated_seat.value = 0;
//     hazard_lights_misc_cmd_.fl_vented_seat.value = 0;
//     hazard_lights_misc_cmd_.fr_vented_seat.value = 0;

//   }


void BelivVehInterface::callbackBrakeRpt(
  const ds_dbw_msgs::msg::BrakeReport::ConstSharedPtr rpt)
{
  sub_brake_ptr_ = rpt;
}

void BelivVehInterface::onControlModeRequest(
  const ControlModeCommand::Request::SharedPtr request,
  const ControlModeCommand::Response::SharedPtr response)
{
  if (request->mode == ControlModeCommand::Request::AUTONOMOUS) {
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  if (request->mode == ControlModeCommand::Request::MANUAL) {
    is_clear_override_needed_ = true;
    response->success = true;
    return;
  }

  RCLCPP_ERROR(get_logger(), "unsupported control_mode!!");
  response->success = false;
  return;
}

void BelivVehInterface::callbackEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

void BelivVehInterface::recvDbwEnabled(std_msgs::msg::Bool::ConstSharedPtr msg){
  dbw_enable_ = msg->data;
}


int32_t BelivVehInterface::toAutowareTurnIndicatorsReport(
  const ds_dbw_msgs::msg::TurnSignalReport &turn_signal_rpt)
{
  using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  using ds_dbw_msgs::msg::TurnSignalReport;
  using ds_dbw_msgs::msg::TurnSignal;

  if (turn_signal_rpt.output.value == ds_dbw_msgs::msg::TurnSignal::RIGHT){
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (turn_signal_rpt.output.value == ds_dbw_msgs::msg::TurnSignal::LEFT){
        return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (turn_signal_rpt.output.value == ds_dbw_msgs::msg::TurnSignal::NONE){
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}



int32_t BelivVehInterface::toAutowareHazardLightsReport(
  const ds_dbw_msgs::msg::TurnSignalReport &turn_signal_rpt)
{
  using autoware_vehicle_msgs::msg::HazardLightsReport;
  using ds_dbw_msgs::msg::TurnSignalReport;
  using ds_dbw_msgs::msg::TurnSignal;

  if (turn_signal_rpt.output.value == ds_dbw_msgs::msg::TurnSignal::HAZARD) {
    return HazardLightsReport::ENABLE;
  }

  return HazardLightsReport::DISABLE;
}

void BelivVehInterface::publishCommands(){
  /* guard */
/*   if (sub_steering_ptr_ || sub_gear_ptr_ || sub_turn_signal_ptr_ || sub_ulc_rpt_ptr_){
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "sub_steering_ptr = %d, sub_gear_ptr= %d, sub_misc_ptr=%d, sub_ulc_rpt_ptr=%d",
      sub_steering_ptr_ !=nullptr, sub_gear_ptr_!=nullptr, sub_turn_signal_ptr_!=nullptr, sub_ulc_rpt_ptr_!=nullptr
    );
  } */
  // if (publish_turn_indicators_ == true) {
  //   RCLCPP_INFO(get_logger(), "publish_turn_indicators_ = true");
  //   pub_turn_signal_cmd_->publish(turn_signal_cmd_);
  // }
  // else {
  //   RCLCPP_INFO(get_logger(), "publish_turn_indicators_ = false");
  //   pub_turn_signal_cmd_->publish(hazard_lights_misc_cmd_);
  // }
  pub_turn_signal_cmd_->publish(turn_signal_cmd_);
  pub_ulc_cmd_->publish(ulc_cmd_);
  pub_steering_cmd_->publish(steering_cmd_);

}
}

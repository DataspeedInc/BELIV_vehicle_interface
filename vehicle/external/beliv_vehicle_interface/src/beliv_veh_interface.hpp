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
#ifndef BELIV_VEH_INTERFACE__BELIV_VEH_INTERFACE_HPP_
#define BELIV_VEH_INTERFACE__BELIV_VEH_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

// Messages
#include <can_msgs/msg/frame.hpp>
#include <ds_dbw_msgs/msg/brake_cmd.hpp>
#include <ds_dbw_msgs/msg/brake_info.hpp>
#include <ds_dbw_msgs/msg/brake_report.hpp>
// #include <ds_dbw_msgs/msg/driver_assist_report.hpp>
// #include <ds_dbw_msgs/msg/fuel_level_report.hpp>
#include <ds_dbw_msgs/msg/gear_cmd.hpp>
#include <ds_dbw_msgs/msg/gear_report.hpp>
#include <ds_dbw_msgs/msg/misc_cmd.hpp>
#include <ds_dbw_msgs/msg/misc_report.hpp>
#include <ds_dbw_msgs/msg/steering_cmd.hpp>
#include <ds_dbw_msgs/msg/steering_report.hpp>
// #include <ds_dbw_msgs/msg/surround_report.hpp>
#include <ds_dbw_msgs/msg/throttle_cmd.hpp>
#include <ds_dbw_msgs/msg/throttle_info.hpp>
#include <ds_dbw_msgs/msg/throttle_report.hpp>
#include <ds_dbw_msgs/msg/tire_pressures.hpp>
#include <ds_dbw_msgs/msg/turn_signal.hpp>
#include <ds_dbw_msgs/msg/wheel_positions.hpp>
#include <ds_dbw_msgs/msg/wheel_speeds.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/empty.hpp>
#include <ds_dbw_msgs/msg/ulc_cmd.hpp>
#include <ds_dbw_msgs/msg/ulc_report.hpp>



// The following messages are deprecated
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// Platform and module version map
#include <dataspeed_dbw_common/PlatformMap.hpp>

//For Autoware

#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
// #include <autoware_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <tier4_api_msgs/msg/door_status.hpp>
#include <tier4_external_api_msgs/srv/set_door.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>


namespace ds_dbw_can { 
class BelivVehInterface : public rclcpp::Node
{
public:
    using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
    using ActuationStatusStamped = tier4_vehicle_msgs::msg::ActuationStatusStamped;
    using SteeringWheelStatusStamped = tier4_vehicle_msgs::msg::SteeringWheelStatusStamped;
    using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
    // using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
    using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
    BelivVehInterface();

private:

    typedef message_filters::sync_policies::ApproximateTime<
        ds_dbw_msgs::msg::SteeringReport, ds_dbw_msgs::msg::GearReport,
        ds_dbw_msgs::msg::MiscReport, ds_dbw_msgs::msg::UlcReport>
        BelivFeedbacksSyncPolicy; 

    /* parameters */
    std::string base_frame_id_;
    int command_timeout_ms_;  // vehicle_cmd timeout [ms]
    bool dbw_enable_ =false;
    bool is_dbw_rpt_received_ = false;
    bool is_clear_override_needed_ = false;
    bool prev_override_ = false;
    // bool publish_turn_indicators_ = true;
    double loop_rate_;           // [Hz]
    double wheel_base_;
    double steering_ratio_;
    double acker_wheelbase_; // 112.2 inches
    double track_width;

    /* Subscription */
    // from Autoware
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_cmd_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr sub_gear_cmd_;
    rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr
        sub_turn_indicators_cmd_;
    // rclcpp::Subscription<HazardLightsCommand>::SharedPtr
    //     sub_hazard_lights_cmd_;
    //rclcpp::Subscription<ActuationCommandStamped>::SharedPtr sub_actuation_cmd_;
    rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr sub_emergency_;
    
    // from dbwNode
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
    rclcpp::Subscription<ds_dbw_msgs::msg::BrakeReport>::SharedPtr sub_brake_;
    rclcpp::Subscription<ds_dbw_msgs::msg::ThrottleReport>::SharedPtr sub_throttle_;
    std::unique_ptr<message_filters::Subscriber<ds_dbw_msgs::msg::SteeringReport>> sub_steering_;
    std::unique_ptr<message_filters::Subscriber<ds_dbw_msgs::msg::GearReport>> sub_gear_;
    std::unique_ptr<message_filters::Subscriber<ds_dbw_msgs::msg::MiscReport>> sub_misc_1_;
    rclcpp::Subscription<ds_dbw_msgs::msg::WheelSpeeds>::SharedPtr sub_wheel_speeds_;
    rclcpp::Subscription<ds_dbw_msgs::msg::WheelPositions>::SharedPtr sub_wheel_positions_;
    rclcpp::Subscription<ds_dbw_msgs::msg::TirePressures>::SharedPtr sub_tire_pressure_;
    // rclcpp::Subscription<ds_dbw_msgs::msg::FuelLevelReport>::SharedPtr sub_fuel_level_;
    // rclcpp::Subscription<ds_dbw_msgs::msg::SurroundReport>::SharedPtr sub_surround_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar_cloud_;
    rclcpp::Subscription<ds_dbw_msgs::msg::BrakeInfo>::SharedPtr sub_brake_info_;
    rclcpp::Subscription<ds_dbw_msgs::msg::ThrottleInfo>::SharedPtr sub_throttle_info_;
    // rclcpp::Subscription<ds_dbw_msgs::msg::DriverAssistReport>::SharedPtr sub_driver_assist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_gps_vel_;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sub_gps_time_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_vin_;      // Deprecated message
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_sys_enable_; // Deprecated message
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;

    // from UlcNode
    std::unique_ptr<message_filters::Subscriber<ds_dbw_msgs::msg::UlcReport>> sub_ulc_rpt_;
    //rclcpp::Subscription<ds_dbw_msgs::UlcReport>::SharedPtr sub_ulc_rpt;

    std::unique_ptr<message_filters::Synchronizer<BelivFeedbacksSyncPolicy>> beliv_feedbacks_sync_;
    
    /* Publisher */
    // To UlcNode
    rclcpp::Publisher<ds_dbw_msgs::msg::UlcCmd>::SharedPtr pub_ulc_cmd_;
    // To dbwNode
    rclcpp::Publisher<ds_dbw_msgs::msg::SteeringCmd>::SharedPtr pub_steering_cmd_;
    rclcpp::Publisher<ds_dbw_msgs::msg::MiscCmd>::SharedPtr pub_misc_cmd_;
    // rclcpp::Publisher<ds_dbw_msgs::msg::MiscCmd>::SharedPtr pub_hazard_lights_cmd_;


    // To Autoware
    rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr pub_battery_status_; // added a publisher for battery status
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
        pub_control_mode_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_vehicle_twist_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
        pub_steering_status_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr pub_gear_status_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
        pub_turn_indicators_status_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
        pub_hazard_lights_status_;
    //rclcpp::Publisher<ActuationStatusStamped>::SharedPtr pub_actuation_status_;
    rclcpp::Publisher<SteeringWheelStatusStamped>::SharedPtr pub_steering_wheel_status_;
    rclcpp::Publisher<tier4_api_msgs::msg::DoorStatus>::SharedPtr pub_door_status_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    //service
    //tier4_api_utils::Service<tier4_external_api_msgs::srv::SetDoor>::SharedPtr srv_;
    rclcpp::Service<ControlModeCommand>::SharedPtr control_mode_server_;

    //ds_dbw_msgs::msg::UlcCmd ulc_cmd_;

      /* input values */
    ActuationCommandStamped::ConstSharedPtr actuation_cmd_ptr_;
    autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;
    autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
    autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
    autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
 
    ds_dbw_msgs::msg::SteeringReport::ConstSharedPtr sub_steering_ptr_;
    ds_dbw_msgs::msg::GearReport::ConstSharedPtr sub_gear_ptr_;
    ds_dbw_msgs::msg::MiscReport::ConstSharedPtr sub_misc_ptr_;
    ds_dbw_msgs::msg::UlcReport::ConstSharedPtr sub_ulc_rpt_ptr_;
    ds_dbw_msgs::msg::BrakeReport::ConstSharedPtr sub_brake_ptr_;
    ds_dbw_msgs::msg::UlcCmd ulc_cmd_;
    ds_dbw_msgs::msg::SteeringCmd steering_cmd_;
    ds_dbw_msgs::msg::MiscCmd turn_indicators_misc_cmd_;
    // ds_dbw_msgs::msg::MiscCmd hazard_lights_misc_cmd_;

    bool is_emergency_{false};
    rclcpp::Time control_command_received_time_;
    rclcpp::Time turn_indicators_misc_command_received_time_;
    // rclcpp::Time hazard_lights_misc_command_received_time_;

   
    void callbackControlCmd(
        const autoware_control_msgs::msg::Control& msg);
    void callbackTurnIndicatorsCmd(
        const TurnIndicatorsCommand& turning_indicators_cmd);
    // void callbackHazardLightsCmd(
    //     const HazardLightsCommand& hazard_lights_cmd);
    void callbackBrakeRpt(const ds_dbw_msgs::msg::BrakeReport::ConstSharedPtr rpt);
    void callbackInterface(
        const ds_dbw_msgs::msg::SteeringReport::ConstSharedPtr steering_rpt,
        const ds_dbw_msgs::msg::GearReport::ConstSharedPtr gear_rpt,
        const ds_dbw_msgs::msg::MiscReport::ConstSharedPtr misc_rpt,
        const ds_dbw_msgs::msg::UlcReport::ConstSharedPtr ulc_rpt);
    int32_t toAutowareShiftReport(const ds_dbw_msgs::msg::GearReport& gear_rpt);
    int32_t toAutowareTurnIndicatorsReport(const ds_dbw_msgs::msg::MiscReport &misc_rpt);
    int32_t toAutowareHazardLightsReport(const ds_dbw_msgs::msg::MiscReport &misc_rpt);
    void onControlModeRequest(
        const ControlModeCommand::Request::SharedPtr request,
        const ControlModeCommand::Response::SharedPtr response);
    void callbackEmergencyCmd(
        const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
    void recvDbwEnabled(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void publishCommands();

};
}
#endif
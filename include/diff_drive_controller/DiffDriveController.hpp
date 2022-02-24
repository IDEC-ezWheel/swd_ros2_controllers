/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.hpp
 */

#include "chrono"
#include "diff_drive_controller/DiffDriveParameters.hpp"
#include "ezw-smc-service/DBusClient.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "iostream"
#include "lifecycle_msgs/msg/transition.hpp"
#include "memory"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "string"
#include "swd_ros2_controllers/msg/safety_functions.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "thread"
#include "utility"

#define M_MAX(a, b) ((a) > (b) ? (a) : (b))
#define M_MIN(a, b) ((a) < (b) ? (a) : (b))
#define M_SIGN(a) ((a) > 0 ? 1 : -1)
#define M_BOUND_ANGLE(a) (((a) > M_PI) ? ((a)-2. * M_PI) : (((a) < -M_PI) ? ((a) + 2. * M_PI) : (a)))

namespace ezw {
    namespace swd {

        /**
         * @brief Differential Drive Controller for ez-Wheel Gen2 wheels
         * It subscribes to the `/node/set_speed` or `/node/cmd_vel` or `/node/soft_brake` topics:
         * - `/node/set_speed` of type `geometry_msgs::msg::Point`: The `x`, `y`
         *   components of the `/set_speed` message
         *   represents respectively the left and right motor speed in (rad/s)
         * - `/node/cmd_vel` of type `geometry_msgs::msg::Twist`: The linear and angular
         *   velocities
         * - `/node/soft_brake` of type `std_msgs::msg::Bool`: To active or release the brake.
         * The controller publishes :
         * - the odometry to `/node/odom`, type `nav_msgs::msg::Odometry`
         * - TFs, the safety functions to `/node/safety`, type `swd_ros2_controllers::msg::SafetyFunctions`.
        **/
        class DiffDriveController : public rclcpp::Node {
           public:
            /**
             * @brief Construct a new Diff Drive Controller object
             * 
             * @param p_node_name Node name
             * @param p_params  
             */
            explicit DiffDriveController(const std::string &p_node_name, const std::shared_ptr<const DiffDriveParameters> p_params);

           private:
            // Publishers
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_odom;
            rclcpp::Publisher<swd_ros2_controllers::msg::SafetyFunctions>::SharedPtr m_pub_safety;
            // Subscribers
            rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr m_sub_command_set_speed;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_command_cmd_vel;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_brake;

            // TF broadcaster
            std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf2_br;

            // Callbacks
            void cbCmdVel(geometry_msgs::msg::Twist::SharedPtr p_cmd_vel);
            void cbTimerOdom(), cbWatchdog(), cbTimerStateMachine(), cbTimerSafety();
            void cbSoftBrake(const std_msgs::msg::Bool &p_msg);
            void cbSetSpeed(const geometry_msgs::msg::Point &p_speed);

            // Change wheels speed
            void setSpeeds(int32_t p_left_speed, int32_t p_right_speed);

            // Timers
            rclcpp::TimerBase::SharedPtr m_timer_odom, m_timer_watchdog, m_timer_pds, m_timer_safety;

            // Mutex
            std::mutex m_safety_msg_mtx;

            // Ezw message : SafetyFunctions
            swd_ros2_controllers::msg::SafetyFunctions m_safety_msg;

            // DiffDriveParameters object
            const std::shared_ptr<const DiffDriveParameters> m_params;

            // Params
            double m_x_prev = 0.0, m_y_prev = 0.0, m_theta_prev = 0.0;
            double m_x_prev_err = 0.0, m_y_prev_err = 0.0, m_theta_prev_err = 0.0;
            int32_t m_dist_left_prev_mm = 0, m_dist_right_prev_mm = 0;
            double m_left_wheel_diameter_m, m_right_wheel_diameter_m, m_l_motor_reduction, m_r_motor_reduction, m_left_encoder_relative_error, m_right_encoder_relative_error;
            int m_left_wheel_polarity, m_max_motor_speed_rpm, m_motor_sls_rpm;
            bool m_nmt_ok, m_pds_ok;
            ezw::smcservice::DBusClient m_left_controller, m_right_controller;
        };

    }  // namespace swd
}  // namespace ezw

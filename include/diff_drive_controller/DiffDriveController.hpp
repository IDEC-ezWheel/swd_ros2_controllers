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
#include "map"
#include "memory"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "string"
#include "swd_ros2_controllers/msg/safety_functions.hpp"
#include "swd_ros2_controllers/msg/wheel_odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "thread"
#include "utility"

#define M_MAX(a, b) ((a) > (b) ? (a) : (b))
#define M_MIN(a, b) ((a) < (b) ? (a) : (b))
#define M_SIGN(a) ((a) > 0 ? 1 : -1)
#define M_BOUND_ANGLE(a) (((a) > M_PI) ? ((a) - 2. * M_PI) : (((a) < -M_PI) ? ((a) + 2. * M_PI) : (a)))

namespace ezw::swd {

    /**
     * @brief Differential Drive Controller for ez-Wheel Safety Wheel Drive.
     * It subscribes to the `/set_speed` or `/cmd_vel` or `/soft_brake` topics:
     * - `/set_speed` of type `geometry_msgs::msg::Point`: the `x` and `y` represent respectively the left and right motor speed in (rad/s)
     * - `/cmd_vel` of type `geometry_msgs::msg::Twist`: The linear and angular velocities
     * - `/soft_brake` of type `std_msgs::msg::Bool`: To active or release the soft brake.
     * It publishes :
     * - the odometry to `/odom`, type `nav_msgs::msg::Odometry` and the associated TFs
     * - the safety functions to `/safety`, type `swd_ros2_controllers::msg::SafetyFunctions`.
     **/
    class DiffDriveController : public rclcpp::Node {
       public:
        /**
         * @brief Construct a new Diff Drive Controller object
         *
         * @param p_node_name Node name
         */
        explicit DiffDriveController(const std::string &p_node_name);

        /**
         * @brief Destroy the Diff Drive Controller object
         *
         */
        virtual ~DiffDriveController();

       private:
        /**
         * @brief Callback for State Machine timer
         *
         */
        void cbTimerStateMachine();

        /**
         * @brief Callback for SoftBrake
         *
         */
        void cbSoftBrake(const std_msgs::msg::Bool::SharedPtr p_msg);

        /**
         * @brief Callback for Odom timer
         *
         */
        void cbTimerOdom();

        /**
         * @brief Change wheel speed (msg.x = left wheel, msg.y = right wheel) [rad/s]
         *
         * @param p_speed
         */
        void cbSetSpeed(const geometry_msgs::msg::Point::SharedPtr p_speed);

        /**
         * @brief Change robot velocity (linear [m/s], angular [rad/s])
         *
         * @param p_cmd_vel
         */
        void cbCmdVel(geometry_msgs::msg::Twist::SharedPtr p_cmd_vel);

        /**
         * @brief Change robot velocity (left in rpm, right in rpm)
         *
         * @param left_speed
         * @param right_speed
         */
        void setSpeeds(int32_t p_left_speed, int32_t p_right_speed);

        /**
         * @brief Callback for Safety timer
         *
         */
        void cbTimerSafety();

        /**
         * @brief Reset the odometry
         *
         * @param p_reset_odometry
         */
        void cbResetOdometry(std_msgs::msg::Empty::SharedPtr p_reset_odometry);

        /**
         * @brief A safety callback which gets activated if no control message
         * have been received since `control_timeout_ms`
         *
         */
        void cbTimerWatchdogReceive();

        // Publishers
        rclcpp::Publisher<swd_ros2_controllers::msg::WheelOdometry>::SharedPtr m_pub_wheel_odometry;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pub_odom;
        rclcpp::Publisher<swd_ros2_controllers::msg::SafetyFunctions>::SharedPtr m_pub_safety;
        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr m_sub_command_set_speed;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_sub_command_cmd_vel;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_sub_brake;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_sub_reset_odometry;

        // TF broadcaster
        std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf2_br;

        // Node Parameters
        std::shared_ptr<DiffDriveParameters> m_params;

        // Timers
        rclcpp::TimerBase::SharedPtr m_timer_odom, m_timer_watchdog, m_timer_pds, m_timer_safety;

        // Mutex for the odom message
        std::mutex m_odom_msg_mtx;

        // Mutex for the SafetyFunctions message
        std::mutex m_safety_msg_mtx;

        // SafetyFunctions message
        swd_ros2_controllers::msg::SafetyFunctions m_safety_msg;

        double m_x_prev = 0.0, m_y_prev = 0.0, m_theta_prev = 0.0;
        double m_x_prev_err = 0.0, m_y_prev_err = 0.0, m_theta_prev_err = 0.0;
        int32_t m_dist_left_prev_mm = 0, m_dist_right_prev_mm = 0;
        int32_t m_left_encoder_offset_mm = 0, m_right_encoder_offset_mm = 0;
        uint64_t m_left_timestamp_prev_us = 0, m_right_timestamp_prev_us = 0;
        double m_left_wheel_diameter_m, m_right_wheel_diameter_m, m_l_motor_reduction, m_r_motor_reduction = 0.0;
        bool m_left_motor_polarity = false;
        bool m_nmt_ok, m_pds_ok = false;
        uint32_t m_left_min_speed_rpm, m_right_min_speed_rpm = 0;

        ezw::smcservice::DBusClient m_left_controller, m_right_controller;

        std::multimap<ezw::smccore::ISafeMotionService::SafetyFunctionId, int8_t> m_left_safety_functions, m_right_safety_functions;
    };
}  // namespace ezw::swd

/**
 * Copyright (C) 2022-2024 ez-Wheel S.A.S.
 *
 * @file DiffDriveParameters.hpp
 */

#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

constexpr auto DEFAULT_BASELINE_M = 0.0;
constexpr auto DEFAULT_LEFT_CONFIG_FILE = "/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini";
constexpr auto DEFAULT_RIGHT_CONFIG_FILE = "/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini";
constexpr auto DEFAULT_PUB_FREQ_HZ = 20;
constexpr auto DEFAULT_WATCHDOG_RECEIVE_MS = 500;
constexpr auto DEFAULT_BASE_FRAME = "base_link";
constexpr auto DEFAULT_ODOM_FRAME = "odom";
constexpr auto DEFAULT_PUBLISH_ODOM = true;
constexpr auto DEFAULT_PUBLISH_TF = true;
constexpr auto DEFAULT_PUBLISH_SAFETY_FCNS = true;
constexpr auto DEFAULT_MOTOR_MAX_SPEED_RPM = 1050;
constexpr auto DEFAULT_MOTOR_MAX_SLS_1_RPM = 560;
constexpr auto DEFAULT_MOTOR_MAX_SLS_2_RPM = 680;
constexpr auto DEFAULT_MOTOR_MAX_DELTA_RPM = DEFAULT_MOTOR_MAX_SPEED_RPM / 2;
constexpr auto DEFAULT_HAVE_BACKWARD_SLS = false;
constexpr auto DEFAULT_LEFT_RELATIVE_ERROR = 0.2;   // 20% of error
constexpr auto DEFAULT_RIGHT_RELATIVE_ERROR = 0.2;  // 20% of error
constexpr auto DEFAULT_ACCURATE_ODOMETRY = false;
constexpr auto DEFAULT_PUBLISH_WHEEL_ODOMETRY = false;
constexpr auto DEFAULT_CHECK_SDI_DURING_TURN_IN_PLACE = false;

namespace ezw::swd {
    using namespace std::chrono_literals;

    /**
     * @brief This class provides the parameters of the swd_diff_drive_controller node.
     *
     */
    class DiffDriveParameters {
       public:
        /**
         * @brief Construct a new Diff Drive Parameters object
         *
         * @param p_node Diff drive node
         */
        explicit DiffDriveParameters(rclcpp::Node* p_node);

        /**
         * @brief Get the Baseline (m)
         *
         * @return double
         */
        auto getBaseline() -> double;

        /**
         * @brief Get the Left Config File
         *
         * @return std::string
         */
        auto getLeftConfigFile() -> std::string;

        /**
         * @brief Get the Right Config File
         *
         * @return std::string
         */
        auto getRightConfigFile() -> std::string;

        /**
         * @brief Get the Pub Freq (Hz)
         *
         * @return int
         */
        auto getPubFreqHz() -> int;

        /**
         * @brief Get the Watchdog Receive (ms)
         *
         * @return int
         */
        auto getWatchdogReceiveMs() -> int;

        /**
         * @brief Get the Base Frame
         *
         * @return std::string
         */
        auto getBaseFrame() -> std::string;

        /**
         * @brief Get the Odom Frame
         *
         * @return std::string
         */
        auto getOdomFrame() -> std::string;

        /**
         * @brief Get the Publish Odom flag
         *
         * @return true if publish enabled
         * @return false
         */
        auto getPublishOdom() -> bool;

        /**
         * @brief Get the Publish Tf
         *
         * @return true if publish enabled
         * @return false
         */
        auto getPublishTf() -> bool;

        /**
         * @brief Get the Publish Safety flag
         *
         * @return true if publish enabled
         * @return false
         */
        auto getPublishSafety() -> bool;

        /**
         * @brief Get the Have Backward SLS flag
         *
         * @return true if a backward SLS is handled by a LIDAR sensor
         * @return false
         */
        auto getHaveBackwardSls() -> bool;

        /**
         * @brief Get the Left Encoder Relative Error
         *
         * @return float
         */
        auto getLeftEncoderRelativeError() -> float;

        /**
         * @brief Get the Right Encoder Relative Error
         *
         * @return float
         */
        auto getRightEncoderRelativeError() -> float;

        /**
         * @brief Get the Motor Max Speed Rpm
         *
         * @return int
         */
        auto getMotorMaxSpeedRpm() -> int;

        /**
         * @brief Get the Motor Max SLS_1 Speed Rpm
         *
         * @return int
         */
        auto getMotorMaxSls1SpeedRpm() -> int;

        /**
         * @brief Get the Motor Max SLS_2 Speed Rpm
         *
         * @return int
         */
        auto getMotorMaxSls2SpeedRpm() -> int;

        /**
         * @brief Get the Motor Max Delta Speed Rpm betwwen the both motors
         *
         * @return int
         */
        auto getMotorMaxDeltaSpeedRpm() -> int;

        /**
         * @brief Get the Accurate Odometry flag
         *
         * @return true if accurate odometry shall be used
         */
        auto getAccurateOdometry() -> bool;

        /**
         * @brief Get the Publish Wheel Odometry flag
         *
         * @return true if wheel odometry shall be published
         */
        auto getPublishWheelOdometry() -> bool;

        /**
         * @brief Get the Check SDI during turn in place flag
         *
         * @return true if SDI signal shall be checked during turn in place
         */
        auto getCheckSDIDuringTurnInPlace() -> bool;

       private:
        rclcpp::Node* m_node;

        double m_baseline_m;
        std::string m_left_config_file, m_right_config_file;
        int m_pub_freq_hz, m_watchdog_receive_ms;
        std::string m_base_frame, m_odom_frame;
        bool m_publish_odom, m_publish_tf, m_publish_safety, m_have_backward_sls;
        float m_left_encoder_relative_error, m_right_encoder_relative_error;
        int m_motor_max_speed_rpm, m_motor_max_sls_1_speed_rpm, m_motor_max_sls_2_speed_rpm, m_motor_max_delta_speed_rpm;
        bool m_accurate_odometry;
        bool m_publish_wheel_odometry;
        bool m_check_sdi_during_turn_in_place;
    };
}  // namespace ezw::swd

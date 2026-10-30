#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "quanser/quanser_thread.h"
#include "quanser/quanser_persistent_stream.h"

#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;

class QBotPlatformDriverInterface : public rclcpp::Node
{
public:
    QBotPlatformDriverInterface()
    : Node("qbot_platform_driver_interface")
    {
        int result;

        other_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = other_cb_group_;

        rclcpp::PublisherOptions pub_options;
        pub_options.callback_group = other_cb_group_;

        // Parameters initialization
        try
        {
            parameter_cb = this->add_on_set_parameters_callback(std::bind(&QBotPlatformDriverInterface::set_parameters_callback, this, _1));
        }
        catch (const std::bad_alloc& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting up parameters callback. %s", e.what());
        }
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

        param_desc.description = "flag for defining if virtual (0) or physical (1)";
        this->declare_parameter("agent_type", 0, param_desc);
        agent_type = this->get_parameter("agent_type").as_int();

        param_desc.description = "Used to modify topic name based on number of agents in a network. It should normally be 0";
        this->declare_parameter("agent_number", 0, param_desc);
        agent_number = this->get_parameter("agent_number").as_int();

        param_desc.description = "Specify a port number for virtual camera in the case of multiagent example. It should normally be 18915";
        this->declare_parameter("port_number", 18888, param_desc);
        port_number = this->get_parameter("port_number").as_int();

        param_desc.description = "Set this to \"true\" to enable, \"false\" to disable the motors.";
        param_desc.additional_constraints = "This arm_mode parameter must be NORMAL (0) to use this parameter";
        this->declare_parameter("arm_robot", true, param_desc);
        arm_robot = this->get_parameter("arm_robot").as_bool();

        param_desc.description = "The speed limit mode of the QBot Platform.";
        param_desc.additional_constraints = "Possible values:\n\t0: Education Speed Limit\n\t1: Research Speed Limit";
        this->declare_parameter("speed_limit_mode", 0, param_desc);
        speed_limit_mode = this->get_parameter("speed_limit_mode").as_int();

        param_desc.description = "The body speed duration (in seconds) of the QBot Platform.";
        this->declare_parameter("body_speed_duration", 0.5, param_desc);
        body_speed_duration = this->get_parameter("body_speed_duration").as_double();

        param_desc.description = "Set this to \"true\" to hold QBot Platform in current position, \"false\" to allow QBot Platform to move.";
        this->declare_parameter("hold", false, param_desc);
        hold = this->get_parameter("hold").as_bool();

        if (agent_type == 0)
        {
            uri = "tcpip://localhost:"+to_string(port_number);
        }
        else if (agent_type == 1)

            // Fixed URI of the physical driver RT model.
            uri = "tcpip://localhost:18888";

        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot form device uri due to invalid agent type!");
        }


        // structures needed for using pstream
        t_pstream_options options;

        options.receive_thread_attributes = NULL;
        options.send_thread_attributes = NULL;

        /* Number of units to be sent/received with each pstream_send or pstream_receive call */
        options.num_receive_units = 17;
        /*
        ** Definition of receive buffer:
        **  0  - wheel 0 encoder (double)
        **  1  - wheel 1 encoder (double)
        **  2  - wheel 0 tachometer (double)
        **  3  - wheel 1 tachometer (double)
        **  4  - motor 0 command voltage (double)
        **  5  - motor 1 command voltage (double)
        **  6  - robot acceleration x (double)
        **  7  - robot acceleration y (double)
        **  8  - robot acceleration z (double)
        **  9  - robot gyro x (double)
        **  10 - robot gyro y (double)
        **  11 - robot gyro z (double)
        **  12 - motor 0 current (double)
        **  13 - motor 1 current (double)
        **  14 - active battery voltage (double)
        **  15 - watchdog triggered (boolean)
        **  16 - feedback timestamp
        */

        options.num_send_units = 10;
        /*
        ** Definition of send buffer:
        **  0 - mode (integer)
        **  1 - user LED control (boolean)
        **  2 - LED red (double)
        **  3 - LED green (double)
        **  4 - LED blue (double)
        **  5 - motor enable (boolean)
        **  6 - motor hold (boolean)
        **  7 - motor 0 speed command / linear velocity (double)
        **  8 - motor 1 speed command / angular velocity (double)
        **  9 - timestamp
        */

        /* Size of each unit to be sent and receive. */
        options.receive_unit_size = sizeof(double);
        options.send_unit_size = sizeof(double);

        options.num_send_dimensions = 0;
        options.num_receive_dimensions = 0;

        /* Send/receive buffer sizes */
        options.receive_buffer_size = 1000;
        options.send_buffer_size = 1000;

        /* The FIFO queue buffer sizes. These FIFO queues will be used if PSTREAM_FLAG_MULTITHREADED is set. */
        options.send_fifo_size = 1000;
        options.receive_fifo_size = 1000;

        /* The persistent stream flags. */
        options.flags = 0;

        /*
        Use blocking I/O. The pstream API is always non-blocking, but setting this flag will create
        a separate thread in which blocking I/O is done. But that thread communicates with the
        pstream_send and pstream_receive via the send/receive FIFO queues. The pstream_send and
        pstream_receive are always non-blocking and setting this flag will not change their mode
        of operation.
        */
        options.flags |= PSTREAM_FLAG_MULTITHREADED;

        /* Set the other flags accordingly. Since we are looking at latency, use the most recent flags */
        options.flags |= PSTREAM_FLAG_MINIMIZE_LATENCY | PSTREAM_FLAG_SEND_MOST_RECENT | PSTREAM_FLAG_RECEIVE_MOST_RECENT;

        /* The size of the options instance */
        options.size = sizeof(options);

        // Setup the stream to get data from the driver model
        result = pstream_connect(uri.c_str(), &options, &client);
        if (result == 0)
        {
            node_running = true;

            // Create the publishers
            speed_feedback_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("qbot_speed_feedback", 1, pub_options);
            battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("qbot_battery", 1, pub_options);
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("qbot_imu", 1, pub_options);
            joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("qbot_joint", 1, pub_options);
            odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("qbot_platform_odometry",1,pub_options);

            // Create the subscribers
            color_subscriber_ = this->create_subscription<std_msgs::msg::ColorRGBA>("qbot_led_strip", 1, std::bind(&QBotPlatformDriverInterface::color_callback, this, _1), sub_options);
            speed_cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&QBotPlatformDriverInterface::speed_command_callback, this, _1), sub_options);
            joint_cmd_subscriber_ = this->create_subscription<control_msgs::msg::JointJog>("qbot_joint_speed_cmd", 1, std::bind(&QBotPlatformDriverInterface::speed_joint_command_callback, this, _1), sub_options);
            timer_ = this->create_wall_timer(driver_comm_sample_time, std::bind(&QBotPlatformDriverInterface::timer_callback, this), timer_cb_group_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Error connecting stream: %d", result);
        }
    }

    ~QBotPlatformDriverInterface()
    {
        int result;
        result = pstream_close(client);
        RCLCPP_INFO(this->get_logger(), "Closing the stream: %d", result);

        node_running = false;
    }

private:
    void timer_callback()
    {
        double receive_buffer[17];
        int result;

        result = pstream_receive(client, receive_buffer);
        if (result >= 0)
        {
            auto start_time = clock_.now();
            auto receive_time = clock_.now();

            // time delta calculation
            rclcpp::Duration delta_time = start_time-end_time_;

            // to get time use delta_time.seconds()

            if (!receive_recovered_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Data received from qbot_driver model.");
                receive_recovered_sent = true;
                receive_warning_sent = false;
            }

            /* Speed Feedback message */
            auto speed_feeback = geometry_msgs::msg::TwistStamped();
            double vR = receive_buffer[2] * wheelRadius;
            double vL = receive_buffer[3] * wheelRadius;

            speed_feeback.header.stamp = receive_time;
            speed_feeback.header.frame_id = "base_link";
            speed_feedback_publisher_->publish(speed_feeback);

            // body speed
            double _body_speed_linear = (vR + vL) / 2.0;
            double _body_speed_angular = (vR - vL) / wheelBase;
            speed_feeback.twist.linear.x = _body_speed_linear;
            speed_feeback.twist.angular.z = _body_speed_angular;

            // body displacement
            auto pose_ = nav_msgs::msg::Odometry();
            _heading_odom = _heading_odom + _body_speed_angular*delta_time.seconds();
            _odometry_x = _odometry_x -(_body_speed_linear/(_body_speed_angular+0.0000001))*sin(_heading_odom)+(_body_speed_linear/(_body_speed_angular+0.0000001))*sin(_heading_odom+_body_speed_angular*delta_time.seconds());
            _odometry_y = _odometry_y + (_body_speed_linear/(_body_speed_angular+0.0000001))*cos(_heading_odom)-(_body_speed_linear/(_body_speed_angular+0.0000001))*cos(_heading_odom+_body_speed_angular*delta_time.seconds());


            double _odom_displacent_x_dt = _odometry_x;
            double _odom_displacent_y_dt = _odometry_y;
            auto x_m = pose_.pose.pose.position.x;
            auto y_m = pose_.pose.pose.position.y;


            // construct quaternion pose representation
            q.setRPY(0.0,0.0,_heading_odom);
            pose_.pose.pose.orientation.x = q.getX();
            pose_.pose.pose.orientation.y = q.getY();
            pose_.pose.pose.orientation.z = q.getZ();
            pose_.pose.pose.orientation.w = q.getW();
            pose_.pose.pose.position.x  = x_m + _odom_displacent_x_dt;
            pose_.pose.pose.position.y  = y_m + _odom_displacent_y_dt;
            pose_.pose.covariance = {
            0.1, 0,    0,    0,    0,    0,
            0,    0.1, 0,    0,    0,    0,
            0,    0,    1e2,  0,    0,    0,
            0,    0,    0,    1e2,  0,    0,
            0,    0,    0,    0,    1e2,  0,
            0,    0,    0,    0,    0,    0.5
            };

            pose_.twist.twist.linear.x  = _body_speed_linear;
            pose_.twist.twist.angular.z = _body_speed_angular;
            pose_.twist.covariance = {
            0.1, 0,    0,    0,    0,    0,
            0,   0.1,  0,    0,    0,    0,
            0,   0,    1e2,  0,    0,    0,
            0,   0,    0,    1e2,  0,    0,
            0,   0,    0,    0,    1e2,  0,
            0,   0,    0,    0,    0,    0.5
            };

            pose_.header.frame_id = "odom";
            pose_.child_frame_id = "base_link";
            pose_.header.stamp = receive_time;
            odometry_pub_->publish(pose_);



            /* Battery State message */
            auto battery_state = sensor_msgs::msg::BatteryState();
            battery_state.voltage = receive_buffer[14];
            battery_state.temperature = std::numeric_limits<double>::quiet_NaN();
            battery_state.current = std::numeric_limits<double>::quiet_NaN();
            battery_state.charge = std::numeric_limits<double>::quiet_NaN();
            battery_state.capacity = std::numeric_limits<double>::quiet_NaN();
            battery_state.design_capacity = 7.0;
            battery_state.percentage = std::numeric_limits<double>::quiet_NaN();
            battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            battery_state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
            battery_state.present = true;

            battery_state.header.stamp = receive_time;
            battery_state.header.frame_id = "base_link";
            battery_state_publisher_->publish(battery_state);

            /* Imu message */
            auto imu = sensor_msgs::msg::Imu();
            imu.linear_acceleration.x = receive_buffer[6];
            imu.linear_acceleration.y = receive_buffer[7];
            imu.linear_acceleration.z = receive_buffer[8];
            imu.angular_velocity.x = receive_buffer[9];
            imu.angular_velocity.y = receive_buffer[10];
            imu.angular_velocity.z = receive_buffer[11];

            imu.header.stamp = receive_time;
            imu.header.frame_id = "imu";
            imu_publisher_->publish(imu);

            /* Joint State message */
            auto joint_state = sensor_msgs::msg::JointState();
            joint_state.position.clear();
            joint_state.position.push_back(receive_buffer[0]);
            joint_state.position.push_back(receive_buffer[1]);
            joint_state.velocity.clear();
            joint_state.velocity.push_back(receive_buffer[2]);
            joint_state.velocity.push_back(receive_buffer[3]);
            joint_state.effort.clear();
            joint_state.effort.push_back(receive_buffer[12]);
            joint_state.effort.push_back(receive_buffer[13]);

            joint_state.header.stamp = receive_time;
            joint_state.header.frame_id = "base_link";
            joint_state_publisher_->publish(joint_state);
            end_time_ = start_time;

        }
        else if (result != -QERR_WOULD_BLOCK)
        {
            if (!receive_warning_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Error (%d) receiving data from qbot_driver model. Make sure the qbot_driver model is running.", result);
                receive_warning_sent = true;
                receive_recovered_sent = false;
            }
        }

        // Preparing the send buffer to be sent to the QBot driver
        speed_limit_mode = this->get_parameter("speed_limit_mode").as_int();

        if (wheel_control)
        {
            if (speed_limit_mode == 0)
                send_buffer[0] = 0.0;   // wheeled education mode
            else
                send_buffer[0] = 2.0;   // wheeled research mode
        }
        else
        {
            if (speed_limit_mode == 0)
                send_buffer[0] = 1.0;   // body education mode
            else
                send_buffer[0] = 3.0;   // body research mode
        }

        if (speed_command_counter_stop > 0)
        {
            // needed to reset the speed command in case we didn't get the speed_command for a while.
            speed_command_counter++;
            if (speed_command_counter > speed_command_counter_stop)
            {
                speed_command_counter = 0;
                send_buffer[7] = 0.0;
                send_buffer[8] = 0.0;
            }
        }

        hold = this->get_parameter("hold").as_bool();
        send_buffer[6] = hold?1.0:0.0;   // hold robot in position

        arm_robot = this->get_parameter("arm_robot").as_bool();
        if (arm_robot)
        {
            send_buffer[5] = 1.0;   // Arm the motor
        }
        else
        {
            send_buffer[5] = 0.0;   // Disarm the motor
            send_buffer[7] = 0.0;   // Set speed command to 0
            send_buffer[8] = 0.0;   // Set speed command to 0
        }

        send_buffer[9] = this->get_clock()->now().seconds();
        result = pstream_send(client, send_buffer);
        if (result < 0)
        {
            if (!send_warning_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Error (%d) sending data to qbot_driver model. Make sure the qbot_driver model is running.", result);
                send_warning_sent = true;
                send_recovered_sent = false;
            }
        }
        else
        {
            if (!send_recovered_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Data sent to qbot_driver model.");
                send_recovered_sent = true;
                send_warning_sent = false;
            }
        }



    }

    void color_callback(const std_msgs::msg::ColorRGBA & led_color)
    {

        send_buffer[1] = led_color.a; // use the "a" as the boolean to use user LED color

        send_buffer[2] = led_color.r;
        send_buffer[3] = led_color.g;
        send_buffer[4] = led_color.b;
    }

    void speed_command_callback(const geometry_msgs::msg::Twist & twist)
    {
        // Handle speed command for the whole robot

        send_buffer[7] = twist.linear.x;
        send_buffer[8] = twist.angular.z;
        std::cout<< "Linear speed is "<<twist.linear.x<<std::endl;


        wheel_control = false;

        // needed to reset the speed command in case we didn't get the speed_command for a while.
        speed_command_counter = 0;

        // convert duration to number of times the communication to driver needs to be set
        body_speed_duration = this->get_parameter("body_speed_duration").as_double();
        speed_command_counter_stop = (t_int)round(body_speed_duration * 1000.0 / 16.0);


    }

    void speed_joint_command_callback(const control_msgs::msg::JointJog & jointjog)
    {
        // Handle speed command for each wheel
        bool got_wheel_command = false;

        for (unsigned int i = 0; i < jointjog.joint_names.size(); i++)
        {
            if (jointjog.joint_names[i].compare("left_wheel") == 0)
            {
                send_buffer[7] = jointjog.velocities[i];
                got_wheel_command = true;
            }
            else if (jointjog.joint_names[i].compare("right_wheel") == 0)
            {
                send_buffer[8] = jointjog.velocities[i];
                got_wheel_command = true;
            }
        }

        // Only change these if a valid wheel command is received.
        if (got_wheel_command)
        {
            wheel_control = true;

            // needed to reset the speed command in case we didn't get the speed_command for a while.
            speed_command_counter = 0;

            // convert duration to number of times the communication to driver needs to be set
            speed_command_counter_stop = (t_int)round(jointjog.duration * 1000.0 / 16.0);

        }
    }

    rcl_interfaces::msg::SetParametersResult set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        result.successful = true;

        // Loop through the parameters....can happen if set_parameters_atomically() is called
        for (const auto & parameter : parameters)
        {
            if (parameter.get_name().compare("agent_type") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare("agent_number") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare("port_number") == 0)
            {
                if (node_running)
                {
                    result.successful = false;
                    result.reason = "Cannot change this parameter while node is running.";
                }
            }
            else if (parameter.get_name().compare("arm_robot") == 0)
            {
                // Can be changed while the node is running.
            }
            else if (parameter.get_name().compare("speed_limit_mode") == 0)
            {
                if ((parameter.as_int() < min_speed_limit_mode) || (parameter.as_int() > max_speed_limit_mode))
                {
                    std::ostringstream error_stream;

                    error_stream << "speed_limit_mode must be between " << min_speed_limit_mode << " and " << max_speed_limit_mode << ".";

                    result.successful = false;
                    result.reason = error_stream.str();
                }
            }
            else if (parameter.get_name().compare("body_speed_duration") == 0)
            {
                if ((parameter.as_double() < 0.0) || (parameter.as_double() > 600.0))
                {
                    std::ostringstream error_stream;

                    error_stream << "body_speed_duration must be between 0 and 600 sec.";

                    result.successful = false;
                    result.reason = error_stream.str();
                }
            }
            else if (parameter.get_name().compare("hold") == 0)
            {
                // Can be changed while the node is running.
            }
            else
            {
                result.successful = false;
                result.reason = "The parameter is invalid.";
            }
        }

        return result;
    }

    rclcpp::CallbackGroup::SharedPtr other_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    // QBot specific publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_feedback_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed_cmd_subscriber_;
    rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_cmd_subscriber_;

    // Odometry specific info
    const t_double wheelRadius = 0.0889 / 2.0;
    const t_double wheelBase = 0.3928;
    float _heading_odom = 0;
    float _prev_right_wheel_meas = 0;
    float _prev_left_wheel_meas = 0;
    float _odometry_x =0;
    float _odometry_y =0;

    tf2::Quaternion q;
    rclcpp::Clock clock_;
    rclcpp::Time end_time_;


    t_pstream client;
    bool receive_warning_sent = false;
    bool receive_recovered_sent = true;
    bool send_warning_sent = false;
    bool send_recovered_sent = true;

    // parameters change callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_cb;


    std::chrono::milliseconds driver_comm_sample_time{16};
    t_int speed_command_counter = 0;
    t_int speed_command_counter_stop = 0;

    bool node_running = false;

    // new parameters for uri and topic names
    string uri ="";
    t_uint32 agent_type;    // = 0;
    t_uint32 agent_number;  // = 0;
    t_uint32 port_number;   // = 0;
    t_double send_buffer[10] = {0.0};

    const t_int min_speed_limit_mode = 0;
    const t_int max_speed_limit_mode = 1;

    // parameters
    t_int speed_limit_mode;
    bool wheel_control = true; // true: speed command to control wheels. false: speed command to control body
    bool arm_robot;
    bool hold;

    t_double body_speed_duration;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS environment
    rclcpp::init(argc, argv);

    // Instantiate the node
    rclcpp::Node::SharedPtr qbp_driver_interface_node = std::make_shared<QBotPlatformDriverInterface>();

    // Get a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(qbp_driver_interface_node);

    RCLCPP_INFO(qbp_driver_interface_node->get_logger(), "Starting qbp_driver_interface loop...");
    executor.spin();
    RCLCPP_INFO(qbp_driver_interface_node->get_logger(), "qbp_driver_interface loop ended.\n");

    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}

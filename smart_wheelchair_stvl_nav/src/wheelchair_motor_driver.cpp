#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>

class WheelchairMotorDriver {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber joy_sub_;
    ros::Publisher odom_pub_;
    serial::Serial motor_serial_;
    
    // Hardware parameters
    double wheel_separation_;
    double wheel_radius_;
    std::string motor_port_;
    int baud_rate_;
    
public:
    WheelchairMotorDriver() : nh_("~") {
        // Load hardware parameters
        nh_.param("wheel_separation", wheel_separation_, 0.6);
        nh_.param("wheel_radius", wheel_radius_, 0.15);
        nh_.param("motor_port", motor_port_, std::string("/dev/ttyUSB0"));
        nh_.param("baud_rate", baud_rate_, 115200);
        
        // Initialize serial communication with motor controllers
        try {
            motor_serial_.setPort(motor_port_);
            motor_serial_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            motor_serial_.setTimeout(timeout);
            motor_serial_.open();
            ROS_INFO("Motor controller connected on %s", motor_port_.c_str());
        } catch (serial::IOException& e) {
            ROS_ERROR("Unable to open motor port: %s", e.what());
        }
        
        // Subscribe to command velocity and joystick
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &WheelchairMotorDriver::cmdVelCallback, this);
        joy_sub_ = nh_.subscribe("/joy", 1, &WheelchairMotorDriver::joyCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // Convert twist to wheel velocities
        double left_vel = (msg->linear.x - msg->angular.z * wheel_separation_ / 2.0) / wheel_radius_;
        double right_vel = (msg->linear.x + msg->angular.z * wheel_separation_ / 2.0) / wheel_radius_;
        
        // Send commands to hardware
        sendMotorCommands(left_vel, right_vel);
    }
    
    void sendMotorCommands(double left_vel, double right_vel) {
        if (motor_serial_.isOpen()) {
            std::string command = "!M " + std::to_string((int)(left_vel * 1000)) + 
                                " " + std::to_string((int)(right_vel * 1000)) + "\r";
            motor_serial_.write(command);
        }
    }
};

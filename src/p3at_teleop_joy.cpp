#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopP3AT 
{
    public:
        TeleopP3AT(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
        
        // Axes of the joystick
        int linear_, angular_;
        double lin_scale_, ang_scale_;

        // Velocity publisher
        ros::Publisher vel_pub_;
        // Joystick subscriber
        ros::Subscriber joy_sub_;
};


// Default constructor for our teleoperator class
TeleopP3AT::TeleopP3AT(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) 
{
    // Set the parameters and take default values if none provided
    nh_priv.param<int>("axis_linear", linear_, 1);
    nh_priv.param<int>("axis_angular", angular_, 2);
    nh_priv.param<double>("scale_angular", ang_scale_, 0.5);
    nh_priv.param<double>("scale_linear", lin_scale_, 0.5);

    // Publisher and subscriber buffer to a queue size of 10 messages 
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    // Subscriber 3rd and 4th arguments are the class method and instance pointed to for the callback
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopP3AT::joyCallback, this);
}

void TeleopP3AT::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    // Create a Twist message to send command velocities
    geometry_msgs::Twist twist_msg;

    // Scale joystick commands to create appropriate velocities
    twist_msg.angular.z = ang_scale_ * joy_msg->axes[angular_];
    twist_msg.linear.x = lin_scale_ * joy_msg->axes[linear_];
    
    vel_pub_.publish(twist_msg);
}


int main(int argc, char** argv)
{
    // Initialise our ROS node
    ros::init(argc, argv, "teleop_p3at");
    // Public & private node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    // Default constructed
    TeleopP3AT teleop_p3at(nh, nh_priv);

    // Spin and leave work for callbacks
    ros::spin();
}

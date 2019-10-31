#include <ros/ros.h>
// Necessary include files for message types
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// Class to teleoperate the P3AT (& other P2OS robots)
class TeleopP3AT 
{
    public:
        // Constructor that takes the public nodehandle to subscribe to and advertise topics
        // As well as other properties related to joystick management
        TeleopP3AT(ros::NodeHandle &nh, int lin, int ang, double ang_s, double lin_s);
        // Callback function for our joystick commands
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
        
    private:   
        // Axes of the joystick
        int linear_, angular_;
        double lin_scale_, ang_scale_;

        // Velocity publisher
        ros::Publisher vel_pub_;
        // Joystick subscriber
        ros::Subscriber joy_sub_;
};


// Constructor for our teleoperator class
TeleopP3AT::TeleopP3AT(ros::NodeHandle &nh, int lin, int ang, double ang_s, double lin_s) 
                : linear_(lin),
                  angular_(ang),
                  ang_scale_(ang_s),
                  lin_scale_(lin_s)
{
    // Publisher and subscriber buffer to a queue size of 10 messages 
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    // Subscriber 3rd and 4th arguments are the callback member function and instance pointed to for the callback
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
    // Public node handle for global namespaces
    ros::NodeHandle nh;
    // Private handle for the current node namespace
    // Access parameters found in /<node_name>/<param_name> e.g. /teleop_p3at/axis_linear 
    ros::NodeHandle nh_priv("~");
    
    // Read parameters from the parameter server (check where they're set in the launch file)
    // Take default values if none are provided
    int linear, angular;
    double ang_scale, lin_scale;
    nh_priv.param<int>("axis_linear", linear, 1);
    nh_priv.param<int>("axis_angular", angular, 2);
    nh_priv.param<double>("scale_angular", ang_scale, 0.5);
    nh_priv.param<double>("scale_linear", lin_scale, 0.5);
    
    // Construct teleoperator
    TeleopP3AT teleop_p3at(nh, linear, angular, ang_scale, lin_scale);

    // Spin and leave work for callbacks
    ros::spin();
}

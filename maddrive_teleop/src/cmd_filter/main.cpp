// ROS
#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>


class SteeringSmoothFilter{

public:

    SteeringSmoothFilter(ros::NodeHandle &nh, double alpha_linear, double alpha_angular)
        : m_alpha_linear(alpha_linear), m_alpha_angular(alpha_angular), m_is_initialized(false) {
        
        sub_ = nh.subscribe("cmd_vel_raw", 100, &SteeringSmoothFilter::callback, this);
        pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

        ROS_INFO_STREAM("Initialized steering filter with alpha_linear: " << alpha_linear << ", alpha_angular: " << alpha_angular);
    }

    void callback(const geometry_msgs::Twist::ConstPtr &msg) {
        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        if (!m_is_initialized) {
            m_prev_linear = linear_velocity;
            m_prev_angular = angular_velocity;
            m_is_initialized = true;
        }

        geometry_msgs::Twist new_msg = *msg;

        double new_linear = m_prev_linear * (1 - m_alpha_linear) + linear_velocity * m_alpha_linear;
        double new_angular = m_prev_angular * (1 - m_alpha_angular) + angular_velocity * m_alpha_angular;

        new_msg.linear.x = m_prev_linear = new_linear;
        new_msg.angular.z = m_prev_angular = new_angular;

        pub_.publish(new_msg);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    bool m_is_initialized;
    double m_prev_linear;
    double m_prev_angular;
    double m_alpha_linear;
    double m_alpha_angular;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "steering_filter");
    ros::NodeHandle nh;

    float steering_alpha_param, linear_alpha_param;
    ros::param::get("~steering_alpha", steering_alpha_param);
    ros::param::get("~linear_alpha", linear_alpha_param);

    SteeringSmoothFilter filter(nh, linear_alpha_param, steering_alpha_param);

    ros::spin();
    return 0;
}
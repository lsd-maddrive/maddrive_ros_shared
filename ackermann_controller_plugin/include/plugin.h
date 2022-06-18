#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/TwistStamped.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <nav_msgs/Odometry.h>


namespace gazebo
{

    template <typename T>
    T clip(const T &n, const T &lower, const T &upper)
    {
        return std::max(lower, std::min(n, upper));
    }

    class AckermannPlugin : public ModelPlugin
    {
    public:
        AckermannPlugin();
        virtual ~AckermannPlugin();

        enum
        {
            FL,
            RL,
            RR,
            FR
        };
        enum
        {
            X,
            Y,
            Z
        };
        enum OdomSource
        {
            ENCODER,
            WORLD
        };

    protected:
        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        virtual void Reset();

    private:
        void onCmdVel(const geometry_msgs::Twist::ConstPtr &cmd);
        void updateCurrentState();
        void updateOdometry(double time_step);

        void twistTimerCallback(const ros::TimerEvent &event);
        void tfTimerCallback(const ros::TimerEvent &event);
        void OnUpdate(const common::UpdateInfo &info);

        void driveUpdate();
        void steeringUpdate(double time_step);

        void stopWheels();

        physics::ModelPtr model;
        GazeboRosPtr gazebo_ros_;
        std::string drive_joint_names_[4];
        std::string steer_joint_names_[4];
        std::vector<physics::JointPtr> steer_joints_, drive_joints_;
        std::vector<double> steer_target_angles_, drive_target_velocities_;
        common::Time last_update_time_;
        double update_period_;

        double wheelbase_;
        double wheel_track_width_;
        double wheel_radius_;

        double target_steering_rad_;
        double target_speed_mps_;

        // PID params
        double drive_p_;
        double drive_i_;
        double drive_d_;
        double drive_imax_;
        double drive_imin_;
        double drive_cmd_max_;
        double steer_p_;
        double steer_i_;
        double steer_d_;
        double steer_imax_;
        double steer_imin_;
        double steer_max_effort_;
        double steer_cmd_max_;

        bool alive_;

        ros::Subscriber sub_vel_cmd_;
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        std::vector<common::PID> steer_PIDs_, drive_PIDs_;

        double cur_virtual_steering_rad_;
        double cur_virtual_speed_rps_;

        std::string command_topic_;
        std::string odom_topic_;

        std::string odom_frame_id_;
        std::string base_frame_id_;

        double update_rate_;
        bool debug_;
        event::ConnectionPtr update_connection_;

        std::vector<double> GetAckAngles(double phi);
        std::vector<double> GetDiffSpeeds(double vel, double phi);
        common::Time GazeboTime();
        void QueueThread();

        // Old

        tf::TransformBroadcaster br_;
        geometry_msgs::Twist twist_;

        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> tf_odom_pub_;

        double mps2rpm;
        double mps2rps;

        double linear_;  //   [m/s]
        double angular_; // [rad/s]

        // Odometry
        double x_;
        double y_;
        double yaw_;
        common::Time last_odom_update_time_;

        // SDF parameters
        std::string robot_name_;
        std::string odom_frame_;
        bool pub_tf_;
        double max_steer_rad_;
        double tf_freq_;

        // Steering values
        double right_angle_;
        double left_angle_;

        double current_steering_angle_;
    };

}

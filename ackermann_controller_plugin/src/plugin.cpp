#include "plugin.h"

#include <boost/assign.hpp>

using namespace std;

#define PLUGIN_NAME "ackermann_plugin"

namespace gazebo
{

    AckermannPlugin::AckermannPlugin()
    {
        target_steering_rad_ = 0.0;
        target_speed_mps_ = 0.0;

        x_ = 0;
        y_ = 0;
        yaw_ = 0;
    }

    std::vector<double> AckermannPlugin::GetAckAngles(double phi)
    {
        std::vector<double> phi_angles;
        double numerator = 2.0 * wheelbase_ * sin(phi);
        phi_angles.assign(4, 0.0);
        phi_angles[FL] = atan2(numerator,
                               (2.0 * wheelbase_ * cos(phi) - wheel_track_width_ * sin(phi)));
        phi_angles[FR] = atan2(numerator,
                               (2.0 * wheelbase_ * cos(phi) + wheel_track_width_ * sin(phi)));
        return phi_angles;
    }

    std::vector<double> AckermannPlugin::GetDiffSpeeds(double vel, double phi)
    {
        // TODO - consider using real phi instead of target
        std::vector<double> wheel_speeds;
        wheel_speeds.assign(4, 0.0);
        wheel_speeds[RL] = vel * (1.0 - (wheel_track_width_ * tan(phi)) /
                                            (2.0 * wheelbase_));
        wheel_speeds[RR] = vel * (1.0 + (wheel_track_width_ * tan(phi)) /
                                            (2.0 * wheelbase_));
        return wheel_speeds;
    }

    void AckermannPlugin::onCmdVel(const geometry_msgs::Twist::ConstPtr &cmd)
    {
        target_steering_rad_ = clip((double)cmd->angular.z, -max_steer_rad_, max_steer_rad_);
        target_speed_mps_ = cmd->linear.x;
        // ROS_INFO_NAMED(PLUGIN_NAME, "Command updated: %.1f, %.1f", target_steering_rad_, target_speed_mps_);
    }

    // TODO - consider using threads for queue topic data
    void AckermannPlugin::QueueThread()
    {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok())
        {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    common::Time AckermannPlugin::GazeboTime()
    {
#if GAZEBO_MAJOR_VERSION >= 8
        return model->GetWorld()->SimTime();
#else
        return model->GetWorld()->GetSimTime();
#endif
    }

    void AckermannPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(model, sdf, "AckermannPlugin"));
        gazebo_ros_->isInitialized();

        this->model = model;

        gazebo_ros_->getParameter<std::string>(drive_joint_names_[RL], "RL_driveJoint", "rear_left_wheel_speed_joint");
        gazebo_ros_->getParameter<std::string>(drive_joint_names_[RR], "RR_driveJoint", "rear_right_wheel_speed_joint");

        gazebo_ros_->getParameter<std::string>(steer_joint_names_[FL], "FL_steerJoint", "front_left_wheel_steer_joint");
        gazebo_ros_->getParameter<std::string>(steer_joint_names_[FR], "FR_steerJoint", "front_right_wheel_steer_joint");

        gazebo_ros_->getParameter<std::string>(robot_name_, "robotName", "");
        gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_vel");
        gazebo_ros_->getParameter<std::string>(odom_topic_, "odomTopic", "wheel_odom");
        gazebo_ros_->getParameter<std::string>(base_frame_id_, "robotBaseFrame", "base_footprint");
        gazebo_ros_->getParameter<std::string>(odom_frame_id_, "odomFrame", "wheel_odom");
        gazebo_ros_->getParameter<double>(wheelbase_, "wheelbase", 0.5);
        gazebo_ros_->getParameter<double>(wheel_radius_, "wheelRadius", 0.25);
        gazebo_ros_->getParameter<double>(wheel_track_width_, "wheelTrackWidth", 0.25);
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);
        gazebo_ros_->getParameter<double>(max_steer_rad_, "maxSteerRad", M_PI * 30 / 180);
        gazebo_ros_->getParameterBoolean(debug_, "debug", false);

        gazebo_ros_->getParameter<double>(steer_p_, "steer_p", 1.0);
        gazebo_ros_->getParameter<double>(steer_i_, "steer_i", 0.0);
        gazebo_ros_->getParameter<double>(steer_d_, "steer_d", 0.0);
        gazebo_ros_->getParameter<double>(steer_imax_, "steer_imax", 1.0);
        gazebo_ros_->getParameter<double>(steer_imin_, "steer_imin", 1.0);
        gazebo_ros_->getParameter<double>(steer_cmd_max_, "steer_max_effort", 20.0);
        gazebo_ros_->getParameter<double>(drive_p_, "drive_p", 1.0);
        gazebo_ros_->getParameter<double>(drive_i_, "drive_i", 0.0);
        gazebo_ros_->getParameter<double>(drive_d_, "drive_d", 0.0);
        gazebo_ros_->getParameter<double>(drive_imax_, "drive_imax", 1.0);
        gazebo_ros_->getParameter<double>(drive_imin_, "drive_imin", 1.0);
        gazebo_ros_->getParameter<double>(drive_cmd_max_, "drive_max_effort", 1.0);

        // Init
        steer_joints_.resize(4);
        drive_joints_.resize(4);
        steer_PIDs_.resize(4);
        drive_PIDs_.resize(4);
        for (int i = 0; i < 4; i++)
        {
            steer_joints_[i] = model->GetJoint(steer_joint_names_[i]);
            drive_joints_[i] = model->GetJoint(drive_joint_names_[i]);

            steer_PIDs_[i].Init(steer_p_, steer_i_, steer_d_, steer_imax_,
                                steer_imin_, steer_cmd_max_, -steer_cmd_max_);
            drive_PIDs_[i].Init(drive_p_, drive_i_, drive_d_, drive_imax_,
                                drive_imin_, drive_cmd_max_, -drive_cmd_max_);

            switch (i)
            {
            case FL:
            case FR:
                steer_target_angles_.push_back(0);
                drive_target_velocities_.push_back(0.0);
                break;
            case RL:
            case RR:
                steer_target_angles_.push_back(0.0);
                drive_target_velocities_.push_back(0);
            }
        }

        // Check required
        assert(drive_joints_[RL]);
        assert(drive_joints_[RR]);
        assert(steer_joints_[FL]);
        assert(steer_joints_[FR]);

        update_period_ = 1. / update_rate_;

        mps2rpm = 60 / wheel_radius_ / (2 * M_PI);
        mps2rps = 1.0 / wheel_radius_;

        // ROS initialization
        ROS_INFO_NAMED(PLUGIN_NAME, "%s: Try to subuscribe to %s",
                       gazebo_ros_->info(), command_topic_.c_str());

        sub_vel_cmd_ = gazebo_ros_->node()->subscribe("cmd_vel", 1, &AckermannPlugin::onCmdVel, this);

        // sub_vel_cmd_ = gazebo_ros_->node()->subscribe(so);
        ROS_INFO_NAMED(PLUGIN_NAME, "%s: Subscribe to %s",
                       gazebo_ros_->info(), command_topic_.c_str());

        // this->callback_queue_thread_ =
        //     boost::thread(boost::bind(&AckermannPlugin::QueueThread, this));

        last_update_time_ = this->GazeboTime();
        update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AckermannPlugin::OnUpdate, this, _1));

        /* Prepare publishers */
        double pose_cov_list[6] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.03};
        double twist_cov_list[6] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.03};

        // Setup odometry realtime publisher + odom message constant fields
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(*gazebo_ros_->node(), odom_topic_, 100));
        odom_pub_->msg_.header.frame_id = odom_frame_id_;
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance = boost::assign::list_of(static_cast<double>(pose_cov_list[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[3]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[4]))(0)(0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[5]));
        odom_pub_->msg_.twist.twist.linear.y = 0;
        odom_pub_->msg_.twist.twist.linear.z = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance = boost::assign::list_of(static_cast<double>(twist_cov_list[0]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[1]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[2]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[3]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[4]))(0)(0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[5]));

        // tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(n_, "/tf", 100));
        // tf_odom_pub_->msg_.transforms.resize(1);
        // tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        // tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
        // tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    }

    void AckermannPlugin::OnUpdate(const common::UpdateInfo &info)
    {
        common::Time current_time = GazeboTime();
        common::Time step_time = current_time - last_update_time_;

        if (step_time > update_period_)
        {
            updateCurrentState();
            updateOdometry(step_time.Double());

            std::vector<double> ack_steer_angles = GetAckAngles(target_steering_rad_);
            std::vector<double> ack_drive_velocities = GetDiffSpeeds(target_speed_mps_, target_steering_rad_);

            // Set target values
            steer_target_angles_[FR] = ack_steer_angles[FR];
            steer_target_angles_[FL] = ack_steer_angles[FL];
            steer_target_angles_[RR] = 0.0;
            steer_target_angles_[RL] = 0.0;
            drive_target_velocities_[FR] = 0.0;
            drive_target_velocities_[FL] = 0.0;
            drive_target_velocities_[RR] = ack_drive_velocities[RR];
            drive_target_velocities_[RL] = ack_drive_velocities[RL];

            double steer_ang_curr, steer_error, steer_cmd_effort;
            double drive_vel_curr, drive_error, drive_cmd_effort;

            for (int i = 0; i < 4; i++)
            {
                switch (i)
                {
                case FL:
                case FR:
                    steer_ang_curr = steer_joints_[i]->Position(X);
                    steer_error = steer_ang_curr - steer_target_angles_[i];
                    steer_cmd_effort = steer_PIDs_[i].Update(steer_error, step_time);
                    steer_joints_[i]->SetForce(X, steer_cmd_effort);
                    break;
                case RL:
                case RR:
                    drive_vel_curr = drive_joints_[i]->GetVelocity(Z) * wheel_radius_;
                    drive_error = drive_vel_curr - drive_target_velocities_[i];
                    drive_cmd_effort = drive_PIDs_[i].Update(drive_error, step_time);
                    drive_joints_[i]->SetForce(Z, drive_cmd_effort);
                }

                if (debug_ && (i == FL || i == FR))
                {
                    double _pe, _ie, _de;
                    // double pGain = steer_PIDs_[i].GetPGain();
                    steer_PIDs_[i].GetErrors(_pe, _ie, _de);
                    ROS_INFO("Steer Joints %i", i);
                    ROS_INFO("\tCurrent angle: %f", steer_ang_curr);
                    ROS_INFO("\tTarget angle: %f", steer_target_angles_[i]);
                    ROS_INFO("\tAngle Error: %f", steer_error);
                    ROS_INFO("\tEffort: %f", steer_cmd_effort);
                    // ROS_INFO("\tP Gain: %f\n", pGain);
                    ROS_INFO("\tP error: %f ", _pe);
                    ROS_INFO("\tI error: %f ", _ie);
                    ROS_INFO("\tD error: %f ", _de);
                }
                if (debug_ && (i == RL || i == RR))
                {
                    double _pe, _ie, _de;
                    // double pGain = drive_PIDs_[i].GetPGain();
                    drive_PIDs_[i].GetErrors(_pe, _ie, _de);
                    ROS_INFO("Drive Joint %i", i);
                    ROS_INFO("\tCurrent Vel: %f", drive_vel_curr);
                    ROS_INFO("\tTarget Vel: %f", drive_target_velocities_[i]);
                    ROS_INFO("\tVel Error: %f", drive_error);
                    ROS_INFO("\tEffort: %f", drive_cmd_effort);
                    // ROS_INFO("\tP Gain: %f\n", pGain);
                    ROS_INFO("\tP error: %f ", _pe);
                    ROS_INFO("\tI error: %f ", _ie);
                    ROS_INFO("\tD error: %f ", _de);
                }
            }

            last_update_time_ = current_time;
        }
    }

    void AckermannPlugin::updateOdometry(double time_step)
    {
        const double linear = cur_virtual_speed_rps_ / mps2rps * time_step;
        const double angular = linear * tan(cur_virtual_steering_rad_) / wheelbase_;
        const double curvature_radius = wheelbase_ / cos(M_PI / 2.0 - cur_virtual_steering_rad_);

        linear_ = linear;
        angular_ = angular;

        if (fabs(curvature_radius) > 0.0001)
        {
            const double elapsed_distance = linear;
            const double elapsed_angle = elapsed_distance / curvature_radius;
            const double x_curvature = curvature_radius * sin(elapsed_angle);
            const double y_curvature = curvature_radius * (cos(elapsed_angle) - 1.0);
            const double wheel_heading = yaw_ + cur_virtual_steering_rad_;
            y_ += x_curvature * sin(wheel_heading) + y_curvature * cos(wheel_heading);
            x_ += x_curvature * cos(wheel_heading) - y_curvature * sin(wheel_heading);
            yaw_ += elapsed_angle;
        }

        const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(yaw_));

        if (odom_pub_->trylock())
        {
            odom_pub_->msg_.header.stamp = ros::Time(last_update_time_.Double());
            odom_pub_->msg_.pose.pose.position.x = x_ + wheelbase_ * (1.0 - cos(yaw_));
            odom_pub_->msg_.pose.pose.position.y = y_ - wheelbase_ * sin(yaw_);
            odom_pub_->msg_.pose.pose.orientation = orientation;
            odom_pub_->msg_.twist.twist.linear.x = 0;
            odom_pub_->msg_.twist.twist.angular.z = 0;
            odom_pub_->unlockAndPublish();
        }

        // if (tf_odom_pub_->trylock())
        // {
        //     geometry_msgs::TransformStamped &odom_frame = tf_odom_pub_->msg_.transforms[0];
        //     odom_frame.header.stamp = ros::Time(last_update_time_.Double());
        //     odom_frame.transform.translation.x = x_ + wheelbase_ * (1.0 - cos(yaw_));
        //     odom_frame.transform.translation.y = y_ - wheelbase_ * sin(yaw_);
        //     odom_frame.transform.rotation = orientation;
        //     tf_odom_pub_->unlockAndPublish();
        // }
    }

    void AckermannPlugin::updateCurrentState()
    {
        double t_cur_lsteer = tan(steer_joints_[FL]->Position(X));
        double t_cur_rsteer = tan(steer_joints_[FR]->Position(X));

        // std::atan(wheelbase_ * std::tan(steering_angle)/std::abs(wheelbase_ + it->lateral_deviation_ * std::tan(steering_angle)));
        double virt_lsteer = atan(wheelbase_ * t_cur_lsteer / (wheelbase_ + t_cur_lsteer * 0.5 * wheel_track_width_));
        double virt_rsteer = atan(wheelbase_ * t_cur_rsteer / (wheelbase_ - t_cur_rsteer * 0.5 * wheel_track_width_));

        cur_virtual_steering_rad_ = (virt_lsteer + virt_rsteer) / 2;
        cur_virtual_speed_rps_ = (drive_joints_[RL]->GetVelocity(Z) + drive_joints_[RR]->GetVelocity(Z)) / 2;
    }

    void AckermannPlugin::Reset()
    {
    }

    AckermannPlugin::~AckermannPlugin()
    {
    }

    GZ_REGISTER_MODEL_PLUGIN(AckermannPlugin)

} // namespace gazebo

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        (void)controller_nh;

        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        cmd_vel_sub_ = root_nh.subscribe(
            "/cmd_vel",1, &HeroChassisController::cmdVelCallback, this);

        return true;
    }

    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
        ROS_INFO_THROTTLE(1.0, "cmd_vel: vx=%.3f vy=%.3f wz=%.3f", vx_, vy_, wz_);
    }
    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
        (void)time;
        (void)period;

        //IK解算出4个轮子角速度
        const double R = lx_ + ly_;

        w_fl_ = (vx_ - vy_ - R * wz_) / wheel_radius_;
        w_fr_ = (vx_ + vy_ + R * wz_) / wheel_radius_;
        w_bl_ = (vx_ + vy_ - R * wz_) / wheel_radius_;
        w_br_ = (vx_ - vy_ + R * wz_) / wheel_radius_;
        //发送信息
        ROS_INFO_THROTTLE(1.0,"cmd_vel: vx=%.3f vy=%.3f wz=%.3f | "
                      "w FL=%.3f FR=%.3f BL=%.3f BR=%.3f",
                      vx_, vy_, wz_, w_fl_, w_fr_, w_bl_, w_br_);


        front_left_joint_.setCommand(w_fl_);
        front_right_joint_.setCommand(w_fr_);
        back_left_joint_.setCommand(w_bl_);
        back_right_joint_.setCommand(w_br_);
    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                           controller_interface::ControllerBase)

    }  // namespace hero_chassis_controller

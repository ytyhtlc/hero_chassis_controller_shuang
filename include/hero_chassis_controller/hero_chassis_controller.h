#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>


namespace hero_chassis_controller {

    class HeroChassisController
        : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;
        //pid启动！
        void starting(const ros::Time &time) override;

        hardware_interface::JointHandle front_left_joint_, front_right_joint_,
            back_left_joint_, back_right_joint_;
    private:
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

        ros::Subscriber cmd_vel_sub_;
        double vx_{0.0};
        double vy_{0.0};
        double wz_{0.0};
        //底盘参数
        double wheel_radius_{0.07625};
        double lx_{0.2};
        double ly_{0.2};
        //4个轮的角速度
        double w_fl_{0.0};
        double w_fr_{0.0};
        double w_bl_{0.0};
        double w_br_{0.0};
        //4个轮各自的PID
        control_toolbox::Pid pid_fl_;
        control_toolbox::Pid pid_fr_;
        control_toolbox::Pid pid_bl_;
        control_toolbox::Pid pid_br_;
    };

}  // namespace hero_chassis_controller

#endif
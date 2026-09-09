#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {


        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        cmd_vel_sub_ = root_nh.subscribe(
            "/cmd_vel",1, &HeroChassisController::cmdVelCallback, this);

        //PID初始化判断
        if (!pid_fl_.init(ros::NodeHandle(controller_nh,"pid_fl")) ||
            !pid_fr_.init(ros::NodeHandle(controller_nh,"pid_fr")) ||
            !pid_br_.init(ros::NodeHandle(controller_nh,"pid_br")) ||
            !pid_bl_.init(ros::NodeHandle(controller_nh,"pid_bl")) ) {

            ROS_ERROR("Failed to init  wheel PIDS");
            return false;
        }

        //初始化ik期望和实际速度发布者
        pub_fl_des_ = controller_nh.advertise<std_msgs::Float64>("fl/des", 1);
        pub_fl_act_ = controller_nh.advertise<std_msgs::Float64>("fl/act", 1);
        pub_fr_des_ = controller_nh.advertise<std_msgs::Float64>("fr/des", 1);
        pub_fr_act_ = controller_nh.advertise<std_msgs::Float64>("fr/act", 1);
        pub_bl_des_ = controller_nh.advertise<std_msgs::Float64>("bl/des", 1);
        pub_bl_act_ = controller_nh.advertise<std_msgs::Float64>("bl/act", 1);
        pub_br_des_ = controller_nh.advertise<std_msgs::Float64>("br/des", 1);
        pub_br_act_ = controller_nh.advertise<std_msgs::Float64>("br/act", 1);
        return true;
    }
    //PID启动！
    void HeroChassisController::starting(const ros::Time &time) {
        (void)time;
        pid_fl_.reset();
        pid_fr_.reset();
        pid_br_.reset();
        pid_bl_.reset();
    }

    //接收底盘速度消息回调
    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
        ROS_INFO_THROTTLE(1.0, "cmd_vel: vx=%.3f vy=%.3f wz=%.3f", vx_, vy_, wz_);
    }
    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
        (void)time;


        //接收的底盘速度来IK解算出4个轮子目标角速度
        const double R = lx_ + ly_;

        w_fl_ = (vx_ - vy_ - R * wz_) / wheel_radius_;
        w_fr_ = (vx_ + vy_ + R * wz_) / wheel_radius_;
        w_bl_ = (vx_ + vy_ - R * wz_) / wheel_radius_;
        w_br_ = (vx_ - vy_ + R * wz_) / wheel_radius_;

        //计算目标角速度与实际的误差
        const double e_fl = w_fl_ - front_left_joint_.getVelocity();
        const double e_fr = w_fr_ - front_right_joint_.getVelocity();
        const double e_bl = w_bl_ - back_left_joint_.getVelocity();
        const double e_br = w_br_ - back_right_joint_.getVelocity();

        //输出pid计算后的力矩
        front_left_joint_.setCommand(pid_fl_.computeCommand(e_fl,period));
        front_right_joint_.setCommand(pid_fr_.computeCommand(e_fr,period));
        back_left_joint_.setCommand(pid_bl_.computeCommand(e_bl,period));
        back_right_joint_.setCommand(pid_br_.computeCommand(e_br,period));

        //发布4个轮分别的期望和实际速度的消息
        std_msgs::Float64 msg;
        msg.data = w_fl_;
        pub_fl_des_.publish(msg);
        msg.data = front_left_joint_.getVelocity();
        pub_fl_act_.publish(msg);
        msg.data = w_fr_;
        pub_fr_des_.publish(msg);
        msg.data = front_right_joint_.getVelocity();
        pub_fr_act_.publish(msg);
        msg.data = w_bl_;
        pub_bl_des_.publish(msg);
        msg.data = back_left_joint_.getVelocity();
        pub_bl_act_.publish(msg);
        msg.data = w_br_;
        pub_br_des_.publish(msg);
        msg.data = back_right_joint_.getVelocity();
        pub_br_act_.publish(msg);

        //打印实际速度和目标速度日志
        ROS_INFO_THROTTLE(1.0,
        "des FL=%.3f act=%.3f | FR des=%.3f act=%.3f | "
        "BL des=%.3f act=%.3f | BR des=%.3f act=%.3f",
        w_fl_, front_left_joint_.getVelocity(),
        w_fr_, front_right_joint_.getVelocity(),
        w_bl_, back_left_joint_.getVelocity(),
        w_br_, back_right_joint_.getVelocity());

    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                           controller_interface::ControllerBase)

    }  // namespace hero_chassis_controller

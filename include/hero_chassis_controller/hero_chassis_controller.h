#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>//用来发ik期望与实际速度的话题消息给rgt

//用来发TF，Odom
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

//全局系
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>


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

        //速度订阅者
        ros::Subscriber cmd_vel_sub_;

        //期望与实际速度的发布者
        ros::Publisher pub_fl_des_, pub_fl_act_;
        ros::Publisher pub_fr_des_, pub_fr_act_;
        ros::Publisher pub_bl_des_, pub_bl_act_;
        ros::Publisher pub_br_des_, pub_br_act_;

        //接收的在base_link的底盘速度
        double vx_{0.0};
        double vy_{0.0};
        double wz_{0.0};

        //底盘参数
        double wheel_radius_{0.07625};
        double lx_{0.2};
        double ly_{0.2};
        //4个轮的目标角速度
        double w_fl_{0.0};
        double w_fr_{0.0};
        double w_bl_{0.0};
        double w_br_{0.0};

        //4个轮各自的PID
        control_toolbox::Pid pid_fl_;
        control_toolbox::Pid pid_fr_;
        control_toolbox::Pid pid_bl_;
        control_toolbox::Pid pid_br_;

        //创建位姿发布者和广播
        ros::Publisher odom_pub_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        //baselink上的车体速度
        double vx_body_{0.0};
        double vy_body_{0.0};
        double omega_body_{0.0};
        //积分出来的odom位姿
        double x_{0.0};
        double y_{0.0};
        double th_{0.0};

        //真正进IK的车体系速度
        double vx_cmd_{0.0};
        double vy_cmd_{0.0};
        double wz_cmd_{0.0};
        //创建缓冲和listener的指针
        tf2_ros::Buffer tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        //全局系参数
        bool use_global_vel_{false};
        std::string global_frame_{"odom"};

    };

}  // namespace hero_chassis_controller

#endif
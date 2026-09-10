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

        //初始化ik期望和实际速度发布者和创建话题
        pub_fl_des_ = controller_nh.advertise<std_msgs::Float64>("fl/des", 1);
        pub_fl_act_ = controller_nh.advertise<std_msgs::Float64>("fl/act", 1);
        pub_fr_des_ = controller_nh.advertise<std_msgs::Float64>("fr/des", 1);
        pub_fr_act_ = controller_nh.advertise<std_msgs::Float64>("fr/act", 1);
        pub_bl_des_ = controller_nh.advertise<std_msgs::Float64>("bl/des", 1);
        pub_bl_act_ = controller_nh.advertise<std_msgs::Float64>("bl/act", 1);
        pub_br_des_ = controller_nh.advertise<std_msgs::Float64>("br/des", 1);
        pub_br_act_ = controller_nh.advertise<std_msgs::Float64>("br/act", 1);

        //初始化odom发布和创建话题
        odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom",10);

        //获取系参数和构建tf_listener
        controller_nh.param("use_global_vel",use_global_vel_,false);
        controller_nh.param("global_frame",global_frame_,std::string("odom"));
        tf_listener_.reset((new tf2_ros::TransformListener(tf_buffer_)));

        return true;
    }
    //PID启动！
    void HeroChassisController::starting(const ros::Time &time) {
        (void)time;
        pid_fl_.reset();
        pid_fr_.reset();
        pid_br_.reset();
        pid_bl_.reset();
        //每次控制器running时都从原点开始
        x_ = 0.0;
        y_ = 0.0;
        th_ = 0.0;
    }

    //接收底盘速度消息回调
    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
        ROS_INFO_THROTTLE(1.0, "cmd_vel: vx=%.3f vy=%.3f wz=%.3f", vx_, vy_, wz_ );
    }
    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {



        //创建一些只读的信息量：底盘物理，4个轮实际角速度
        const double R = lx_ + ly_;                                          //两个x,y半轴距之和
        const double r = wheel_radius_;                                      //麦轮半径
        const double wfl_real = front_left_joint_.getVelocity();             //麦轮实际角速度
        const double wfr_real = front_right_joint_.getVelocity();
        const double wbl_real = back_left_joint_.getVelocity();
        const double wbr_real = back_right_joint_.getVelocity();

        vx_cmd_ = vx_;
        vy_cmd_ = vy_;
        wz_cmd_ = wz_;

        //判断全局系还是底盘系并分解速度
        if (use_global_vel_) {
            geometry_msgs::Vector3Stamped vin, vout;
            vin.header.stamp = ros::Time(0);
            vin.header.frame_id = global_frame_;
            vin.vector.x = vx_;
            vin.vector.y = vy_;
            vin.vector.z = 0.0;
            try {
                vout = tf_buffer_.transform(vin,"base_link");
                vx_cmd_ = vout.vector.x;
                vy_cmd_ = vout.vector.y;
            }catch (const tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1.0, "global vel TF failed: %s", ex.what());
                // vx_cmd_/vy_cmd_ 已等于 vx_/vy_，保持底盘模式回退
            }
        }

///////////////////////////////////////
///逆运动学IK解算+PID控制+base_link速度发布
///////////////////////////////////////

        //IK解算出4个轮的目标速度
        w_fl_ = (vx_cmd_ - vy_cmd_ - R * wz_cmd_) / wheel_radius_;
        w_fr_ = (vx_cmd_ + vy_cmd_ + R * wz_cmd_) / wheel_radius_;
        w_bl_ = (vx_cmd_ + vy_cmd_ - R * wz_cmd_) / wheel_radius_;
        w_br_ = (vx_cmd_ - vy_cmd_ + R * wz_cmd_) / wheel_radius_;

        //计算目标角速度与实际的误差
        const double e_fl = w_fl_ - wfl_real;
        const double e_fr = w_fr_ - wfr_real;
        const double e_bl = w_bl_ - wbl_real;
        const double e_br = w_br_ - wbr_real;

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

///////////////////////////
///正运动学fk+odom+tf广播部分
///////////////////////////

        //FK解算出底盘bask_link下的速度
        vx_body_ = r / 4.0 * (wfl_real + wfr_real + wbl_real + wbr_real);
        vy_body_ = r / 4.0 * (-wfl_real + wfr_real + wbl_real - wbr_real);
        omega_body_ = r / (4.0 * R) * (-wfl_real + wfr_real - wbl_real + wbr_real);

        //用时间片来积分，将bask_link上的速度分解到odom固定轴上再积分出坐标
        const double dt = period.toSec();
        if (dt > 0.0) {
            x_ += (vx_body_ * std::cos(th_) - vy_body_ * std::sin(th_)) * dt;
            y_ += (vx_body_ * std::sin(th_) + vy_body_ * std::cos(th_)) * dt;
            th_ += omega_body_ * dt;
        }

        //计算四元数
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, th_);
        //创建odom消息和发送
        nav_msgs::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(quat);
        odom.twist.twist.linear.x = vx_body_;
        odom.twist.twist.linear.y = vy_body_;
        odom.twist.twist.angular.z = omega_body_;
        odom_pub_.publish(odom);
        //创建tf消息和广播
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = tf2::toMsg(quat);
        tf_broadcaster_.sendTransform(tf_msg);
    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                           controller_interface::ControllerBase)

    }  // namespace hero_chassis_controller

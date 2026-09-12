#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <X11/Xlib.h>
#include <X11/keysym.h>
#undef Bool
#undef True
#undef False
#undef Status
#undef None

#include <std_msgs/Bool.h>
#include <string>



namespace {

bool keyDown(const char keys[32], KeyCode code) {
    if (code == 0) {
        return false;
    }
    return (keys[code / 8] & (1 << (code % 8))) != 0;
}

// winner: +1 / -1 / 0
//先按下的键是赢家，后面按的不干扰，松开后后面还按住的话直接切换
void updateWinner(bool pos, bool neg, int *winner) {
    if (*winner == 1) {
        if (!pos) {
            *winner = neg ? -1 : 0;
        }
    } else if (*winner == -1) {
        if (!neg) {
            *winner = pos ? 1 : 0;
        }
    } else if (pos != neg) {
        *winner = pos ? 1 : -1;
    }
}

}//namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "wasd_teleop");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Display *dpy = XOpenDisplay(nullptr);
    if (dpy == nullptr) {
        ROS_ERROR("wasd_teleop: cannot open X11 display");
        return 1;
    }

    const KeyCode kc_w = XKeysymToKeycode(dpy, XK_w);
    const KeyCode kc_s = XKeysymToKeycode(dpy, XK_s);
    const KeyCode kc_a = XKeysymToKeycode(dpy, XK_a);
    const KeyCode kc_d = XKeysymToKeycode(dpy, XK_d);
    const KeyCode kc_q = XKeysymToKeycode(dpy, XK_q);
    const KeyCode kc_e = XKeysymToKeycode(dpy, XK_e);
    const KeyCode kc_tab = XKeysymToKeycode(dpy, XK_Tab);

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher mode_pub = nh.advertise<std_msgs::Bool>("/chassis_use_global_vel", 1, true);

    bool use_global = false;
    nh.param("/controller/hero_chassis_controller/use_global_vel", use_global, false);

    std_msgs::Bool mode_msg;
    mode_msg.data = use_global;
    mode_pub.publish(mode_msg);

    int win_x = 0;
    int win_y = 0;
    int win_z = 0;
    bool ign_w = false, ign_s = false, ign_a = false, ign_d = false, ign_q = false, ign_e = false;
    bool tab_prev = false;

    ros::Rate rate(50.0);
    ROS_INFO("wasd_teleop: W/S A/D Q/E move, Tab toggle frame (start %s)",
             use_global ? "GLOBAL/odom" : "CHASSIS/base_link");

    while (ros::ok()) {
        char keys[32] = {0};
        XQueryKeymap(dpy, keys);

        const bool raw_w = keyDown(keys, kc_w);
        const bool raw_s = keyDown(keys, kc_s);
        const bool raw_a = keyDown(keys, kc_a);
        const bool raw_d = keyDown(keys, kc_d);
        const bool raw_q = keyDown(keys, kc_q);
        const bool raw_e = keyDown(keys, kc_e);
        const bool raw_tab = keyDown(keys, kc_tab);

        if (!raw_w) {
            ign_w = false;
        }
        if (!raw_s) {
            ign_s = false;
        }
        if (!raw_a) {
            ign_a = false;
        }
        if (!raw_d) {
            ign_d = false;
        }
        if (!raw_q) {
            ign_q = false;
        }
        if (!raw_e) {
            ign_e = false;
        }

        if (raw_tab && !tab_prev) {
            use_global = !use_global;
            mode_msg.data = use_global;
            mode_pub.publish(mode_msg);
            win_x = win_y = win_z = 0;
            ign_w = raw_w;
            ign_s = raw_s;
            ign_a = raw_a;
            ign_d = raw_d;
            ign_q = raw_q;
            ign_e = raw_e;
            ROS_INFO("wasd_teleop: mode -> %s (release keys to move again)",
                     use_global ? "GLOBAL/odom" : "CHASSIS/base_link");

        }
        tab_prev = raw_tab;

        updateWinner(raw_w && !ign_w, raw_s && !ign_s, &win_x);
        updateWinner(raw_a && !ign_a, raw_d && !ign_d, &win_y);
        updateWinner(raw_q && !ign_q, raw_e && !ign_e, &win_z);

        double max_vx = 0.6, max_vy = 0.6, max_vw = 1.2;
        nh.param("/controller/hero_chassis_controller/max_vel_x", max_vx, 0.6);
        nh.param("/controller/hero_chassis_controller/max_vel_y", max_vy, 0.6);
        nh.param("/controller/hero_chassis_controller/max_vel_w", max_vw, 1.2);

        geometry_msgs::Twist cmd;
        cmd.linear.x = static_cast<double>(win_x) * max_vx;
        cmd.linear.y = static_cast<double>(win_y) * max_vy;
        cmd.angular.z = static_cast<double>(win_z) * max_vw;
        cmd_pub.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }

    XCloseDisplay(dpy);
    return 0;
}

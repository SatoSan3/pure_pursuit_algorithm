#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <string>

#include "PurePursuitAlgorithm.hpp"

namespace {
PurePursuitAlgorithm ppa;

float speedVector;
float speedVectorAngle;
float x = 0, y = 0, angle = 0;
} // namespace

class VirtualOmniWheels {
public:
    void init(ros::NodeHandle nh) {
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        positionX = 0;
        positionY = 0;
        machineAngle = 0;
        update();
    }
    void setSpeed(geometry_msgs::Twist vel) {
        pub.publish(vel);
    }
    //オドメトリの情報を更新する
    void update() {
        //オドメトリ(tf)取得の準備
        static tf::TransformListener ln;
        const std::string target_frame = "odom";

        geometry_msgs::PoseStamped source_pose;
        source_pose.header.frame_id = "base_footprint";
        source_pose.pose.orientation.w = 1.0;

        ros::Time time = ros::Time::now();

        static float oldTime = 0;
        float newTime = time.sec + time.nsec / (1000.f * 1000.f * 1000.f);
        float dt = newTime - oldTime;

        static float oldPositionX = 0;
        static float oldPositionY = 0;

        geometry_msgs::PoseStamped target_pose;
        try {
            ln.waitForTransform(source_pose.header.frame_id, target_frame, time, ros::Duration(1.0));
            ln.transformPose(target_frame, source_pose, target_pose);

            positionX = target_pose.pose.position.x;
            positionY = target_pose.pose.position.y;
            machineAngle = atan2f((target_pose.pose.orientation.x + target_pose.pose.orientation.y + target_pose.pose.orientation.z), target_pose.pose.orientation.w) * 2.f;

            speedVector = std::hypot(positionX - oldPositionX, positionY - oldPositionY) / dt;
            speedVectorAngle = atan2f(positionY - oldPositionY, positionX - oldPositionX);

            oldPositionX = positionX;
            oldPositionY = positionY;
            oldTime = newTime;

        } catch (...) {
            ROS_INFO("tf error");
        }
        time_stamp = newTime;
    }
    float time_stamp, positionX, positionY, machineAngle, speed, speedAngle;

private:
    ros::Publisher pub;
};
namespace {
VirtualOmniWheels ow;
}

void receivePath(const nav_msgs::Path& path) {
    std::cout << "receive" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "machine_driver2");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Subscriber sub = nh.subscribe("path/robot_path", 10, receivePath);

    //速度制御(cmd_vel)の準備

    //経路追従関連の初期化
    for (int i = 0; i < ROUTE_SIZE; ++i) {
        // if (i < 314 * 2) {
        //     ppa.cy[i] = static_cast<float>(i) / 2.f;
        //     ppa.cx[i] = sin(static_cast<float>(i) / 100.f) * 150;
        //     angleList[i] = 0;
        // } else {
        //     ppa.cy[i] = static_cast<float>(i) / 2.f;
        //     ppa.cx[i] = 0;
        //     angleList[i] = M_PI * 10 / 4.0 / (ROUTE_SIZE - 314 * 2) * (i - 314 * 2);
        //     //angleList[i] = 0;
        // }
        // ppa.cy[i] = sin(static_cast<float>(i) / 100.f) * 500;
        // ppa.cx[i] = cos(static_cast<float>(i) / 100.f * 2.f) * 500;
        // angleList[i] = 0;

        //砂時計経路
        ppa.cy[i] = sin(static_cast<float>(i) / 100.f) * 5;
        ppa.cx[i] = sin(static_cast<float>(i) / 100.f * 2.f) * 5;

        //往復経路
        // ppa.cy[i] = static_cast<float>(i % 100) * 5 - 5.f * 100.f / 2.f;
        // ppa.cx[i] = 0;

        ppa.angleList[i] = 0.01 * static_cast<float>(i);
        //angleList[i] = 0.1;
    }

    ow.init(nh);

    ppa.init(ow.time_stamp, ow.positionX, ow.positionY, ow.machineAngle);
    ppa.setTargetSpeed(0.4);

    int i = 0;
    while (ros::ok()) {
        ros::spinOnce();
        ow.update();
        // updatePurePursuitAlgorithm();
        ppa.update(ow.time_stamp, ow.positionX, ow.positionY, ow.machineAngle);

        geometry_msgs::Twist cmd_vel = ppa.getCommandVelocity();
        ow.setSpeed(cmd_vel);

        rate.sleep();
    }

    return 0;
}

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <string>

#include "PathReader.hpp"
#include "PurePursuitAlgorithm.hpp"

class VirtualOmniWheels
{
public:
    void init(ros::NodeHandle nh)
    {
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        positionX = 0;
        positionY = 0;
        machineAngle = 0;
        update();
    }
    void setSpeed(geometry_msgs::Twist vel)
    {
        pub.publish(vel);
    }
    //オドメトリの情報を更新する
    void update()
    {
        //オドメトリ(tf)取得の準備
        static tf::TransformListener ln;
        const std::string target_frame = "odom";

        geometry_msgs::PoseStamped source_pose;
        source_pose.header.frame_id = "base_footprint";
        source_pose.pose.orientation.w = 1.0;

        ros::Time time = ros::Time::now();

        static float oldTime = 0;
        time_stamp = time.sec + time.nsec / (1000.f * 1000.f * 1000.f);

        static float oldPositionX = 0;
        static float oldPositionY = 0;

        geometry_msgs::PoseStamped target_pose;
        try {
            ln.waitForTransform(source_pose.header.frame_id, target_frame, time, ros::Duration(1.0));
            ln.transformPose(target_frame, source_pose, target_pose);

            positionX = target_pose.pose.position.x;
            positionY = target_pose.pose.position.y;
            machineAngle = atan2f((target_pose.pose.orientation.x + target_pose.pose.orientation.y + target_pose.pose.orientation.z), target_pose.pose.orientation.w) * 2.f;
        }
        catch (...) {
            ROS_INFO("tf error");
        }
    }
    float time_stamp, positionX, positionY, machineAngle, speed, speedAngle;

private:
    ros::Publisher pub;
};
namespace
{
PurePursuitAlgorithm ppa;
VirtualOmniWheels ow;
} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "machine_driver2");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    ow.init(nh);

    ppa.init(ow.time_stamp, ow.positionX, ow.positionY, ow.machineAngle);
    ppa.setTargetSpeed(0.4);

    int i = 0;

    std::string file_name;
    nh.getParam("test_param", file_name);
    std::cout << "file_name??? = " << file_name << std::endl;

    PathReader path_reader;
    ppa.setPath(path_reader.readPath(file_name));

    while (ros::ok()) {
        ros::spinOnce();

        ow.update();
        ppa.update(ow.time_stamp, ow.positionX, ow.positionY, ow.machineAngle);
        ow.setSpeed(ppa.getCommandVelocity());

        rate.sleep();
    }

    return 0;
}

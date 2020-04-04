#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <pure_pursuit_algorithm/PathAction.h>
#include <actionlib/server/simple_action_server.h>

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
    ros::init(argc, argv, "machine_driver");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    ow.init(nh);

    std::string path_list_name, path_file_directory;
    nh.getParam("path_list_name", path_list_name);
    nh.getParam("path_file_directory", path_file_directory);
    PathReader path_reader;
    path_reader.read_path_file_list(path_file_directory, path_list_name);

    ppa.init(ow.time_stamp, ow.positionX, ow.positionY, ow.machineAngle);
    ppa.setTargetSpeed(0.4);
    ppa.setPath(path_reader.readPath(1));

    // Server server(nh, "route_selection_server", false);
    // server.start();
    // test_action::TestGoalConstPtr current_goal;

    while (ros::ok()) {
        ros::spinOnce();

        // if (server.isNewGoalAvailable()) {
        //     current_goal = server.acceptNewGoal();
        //     printf("Update Goal\n");
        // }

        // if (server.isActive()) {
        //     if (server.isPreemptRequested()) {
        //         server.setAborted();
        //         //server.setPreempted();
        //         printf("Preempt Goal\n");
        //     } else {
        //         if (start_time + ros::Duration(current_goal->duration) < ros::Time::now()) {
        //             server.setSucceeded();
        //             printf("Active: publish result id:%i\n", current_goal->task_id);
        //         } else {
        //             test_action::TestFeedback feedback;
        //             feedback.rate = (ros::Time::now() - start_time).toSec() / current_goal->duration;
        //             server.publishFeedback(feedback);
        //             printf("Active: publish feedback id:%i\n", current_goal->task_id);
        //         }
        //     }
        // }

        ow.update();
        ppa.update(ow.time_stamp, ow.positionX, ow.positionY, ow.machineAngle);
        ow.setSpeed(ppa.getCommandVelocity());

        if (ppa.judgeGoal()) {
            std::cout << "reach goal " << std::endl;
        }

        rate.sleep();
    }

    return 0;
}

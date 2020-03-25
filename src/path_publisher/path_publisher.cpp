#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 10);
    ros::Rate loop_rate(10);

    nav_msgs::Path path;
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = 10;
    ps.pose.position.y = 0;
    ps.pose.orientation.x = 0;
    ps.pose.orientation.y = 0;
    ps.pose.orientation.z = 0;
    ps.pose.orientation.w = 1;

    path.poses.push_back(ps);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path_p", 10);
    // path.poses.push_back()

    while (ros::ok()) {
        // std_msgs::String msg;
        // msg.data = "hello world!";
        // ROS_INFO("publish: %s", msg.data.c_str());
        // chatter_pub.publish(msg);

        path_pub.publish(path);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

#include "DataManager.hpp"

nav_msgs::Path makePath(std::vector<std::vector<double>> path_data) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    for (int i = 0; i < path_data.size(); ++i) {
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = path_data[i][0];
        ps.pose.position.y = path_data[i][1];
        ps.pose.position.z = 0;
        ps.pose.orientation.x = 0;
        ps.pose.orientation.y = 0;
        ps.pose.orientation.z = std::sin(path_data[i][2] / 2.0);
        ps.pose.orientation.w = std::cos(path_data[i][2] / 2.0);

        path.poses.push_back(ps);
    }

    return path;
}
int main(int argc, char** argv) {
    std::cout << "aaaa" << std::endl;
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle n("~");
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 10);
    ros::Rate loop_rate(10);

    int param_data;
    std::string file_name;
    n.getParam("test_param", file_name);
    std::cout << file_name << std::endl;

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("robot_path", 10);
    // path.poses.push_back()

    DataManager dm;
    dm.readFileData(file_name);
    std::vector<std::vector<double>> a = dm.getStoredData();

    for (int i = 0; i < a.size(); i++) {
        for (int j = 0; j < a[i].size(); j++) {
            std::cout << a[i][j] << " , ";
        }
        std::cout << std::endl;
    }

    nav_msgs::Path path = makePath(a);

    while (ros::ok()) {
        path_pub.publish(path);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
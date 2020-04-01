#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

#include "DataManager.hpp"

class PathReader
{
public:
    nav_msgs::Path makePath(std::vector<std::vector<double>> path_data)
    {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
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
    nav_msgs::Path readPath(std::string path_file_name)
    {
        DataManager data_manager;
        data_manager.readFileData(path_file_name);
        std::vector<std::vector<double>> a = data_manager.getStoredData();

        for (int i = 0; i < a.size(); i++) {
            for (int j = 0; j < a[i].size(); j++) {
                std::cout << a[i][j] << " , ";
            }
            std::cout << std::endl;
        }

        nav_msgs::Path path = makePath(a);

        return path;
    }
    nav_msgs::Path readPath(int n)
    {
        return readPath(file_name_list[n]);
    }
    void read_path_file_list(std::string path_file_directory, std::string path_file_list)
    {
        std::ifstream ifs(path_file_directory + path_file_list);

        std::string line;
        while (std::getline(ifs, line)) {
            file_name_list.push_back(path_file_directory + line);
        }
    }

    std::vector<std::string> file_name_list;
};
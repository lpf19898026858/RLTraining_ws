#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

// 为了方便，定义一个简单的Point和Boundary结构体
struct Point3D { double x, y, z; };
struct Boundary { Point3D min; Point3D max; };

class YamlMapPublisher {
public:
    YamlMapPublisher(ros::NodeHandle& nh) : nh_(nh) {
        // 1. 从参数服务器获取参数
        nh_.param<std::string>("yaml_file_path", yaml_file_path_, "/home/lpf/docker_shared/rltraining_ws/src/map_generator/points_of_interest.yaml");
        nh_.param<double>("resolution", map_resolution_, 0.1);
        nh_.param<std::string>("map_topic", map_topic_, "/map");
        nh_.param<std::string>("frame_id", map_frame_id_, "map");

        // 2. 创建一个latched publisher
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);

        // 3. 加载、生成并发布地图
        if (loadAndPublishMap()) {
            ROS_INFO("Map has been published successfully on topic: %s", map_topic_.c_str());
        } else {
            ROS_ERROR("Failed to publish map.");
        }
    }

private:
bool loadAndPublishMap() {
    // --- a. 加载并解析YAML文件 (不变) ---
    YAML::Node world_data;
    try {
        world_data = YAML::LoadFile(yaml_file_path_);
        ROS_INFO("Successfully loaded YAML file from: %s", yaml_file_path_.c_str());
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load or parse YAML file: %s", e.what());
        return false;
    }

    const auto& pois = world_data["points_of_interest"];

    // --- b. 确定地图边界和尺寸 (新逻辑: 根据所有 type=="Wall" 的对象) ---
    double world_min_x = std::numeric_limits<double>::max();
    double world_max_x = std::numeric_limits<double>::lowest();
    double world_min_y = std::numeric_limits<double>::max();
    double world_max_y = std::numeric_limits<double>::lowest();

    bool walls_found = false;
    for (const auto& poi : pois) {
        // ******** 这是修改的核心 ********
        // 只关心 type 为 "Wall" 的对象来确定边界
        if (poi["type"].as<std::string>() == "Wall" && poi["simplified_boundary"]) {
            walls_found = true;
            
            // 更新世界坐标的极值
            world_min_x = std::min(world_min_x, poi["simplified_boundary"]["min"]["x"].as<double>());
            world_max_x = std::max(world_max_x, poi["simplified_boundary"]["max"]["x"].as<double>());
            
            // 同样注意YAML中y轴的反转问题
            world_min_y = std::min(world_min_y, poi["simplified_boundary"]["max"]["y"].as<double>());
            world_max_y = std::max(world_max_y, poi["simplified_boundary"]["min"]["y"].as<double>());
        }
    }

    if (!walls_found) {
        ROS_ERROR("No objects with type 'Wall' found. Cannot determine map boundaries.");
        return false;
    }

    // 给边界增加一点padding
    double padding = 1.0; // 1米
    world_min_x -= padding;
    world_min_y -= padding;
    world_max_x += padding;
    world_max_y += padding;

    int map_width_pixels = static_cast<int>(std::ceil((world_max_x - world_min_x) / map_resolution_));
    int map_height_pixels = static_cast<int>(std::ceil((world_max_y - world_min_y) / map_resolution_));

    ROS_INFO("Map dimensions based on walls: %dpx x %dpx", map_width_pixels, map_height_pixels);

    // --- c. 创建并初始化OccupancyGrid消息 ---
    nav_msgs::OccupancyGrid map_msg;

    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_id_;
    map_msg.info.resolution = map_resolution_;
    map_msg.info.width = map_width_pixels;
    map_msg.info.height = map_height_pixels;
    map_msg.info.origin.position.x = world_min_x;
    map_msg.info.origin.position.y = world_min_y;
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;
    map_msg.data.assign(map_width_pixels * map_height_pixels, 0);

    // --- d. "绘制"障碍物 (不变) ---
    for (const auto& poi : pois) {
        std::string type = poi["type"].as<std::string>();

        // 我们只跳过UAV和Ground
        if (type == "UAV" || type == "Ground" || !poi["simplified_boundary"]) {
            continue;
        }
        
        ROS_INFO("Processing obstacle: %s", poi["name"].as<std::string>().c_str());

        Boundary obs_boundary;
        obs_boundary.min.x = poi["simplified_boundary"]["min"]["x"].as<double>();
        obs_boundary.max.x = poi["simplified_boundary"]["max"]["x"].as<double>();
        obs_boundary.min.y = poi["simplified_boundary"]["max"]["y"].as<double>(); // y是反的
        obs_boundary.max.y = poi["simplified_boundary"]["min"]["y"].as<double>();

      int start_x_px = static_cast<int>((obs_boundary.min.x - world_min_x) / map_resolution_);
        int end_x_px = static_cast<int>((obs_boundary.max.x - world_min_x) / map_resolution_);
        int start_y_px = static_cast<int>((obs_boundary.min.y - world_min_y) / map_resolution_);
        int end_y_px = static_cast<int>((obs_boundary.max.y - world_min_y) / map_resolution_);

        // ********** 错误点 2 (由错误1引发) 现在应该可以正常编译 **********
        for (int y = start_y_px; y < end_y_px; ++y) {
            for (int x = start_x_px; x < end_x_px; ++x) {
                if (x >= 0 && x < map_width_pixels && y >= 0 && y < map_height_pixels) {
                    int index = y * map_width_pixels + x;
                    map_msg.data[index] = 100;
                }
            }
        }
    }

    // --- e. 发布地图 ---
    map_pub_.publish(map_msg);
    return true;
}

private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    std::string yaml_file_path_;
    double map_resolution_;
    std::string map_topic_;
    std::string map_frame_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yaml_to_map_publisher_node");
    ros::NodeHandle nh("~"); // 使用私有NodeHandle获取参数

    YamlMapPublisher map_publisher(nh);
    
    // spin() 使节点保持活动状态，以确保latched publisher正常工作
    // 并允许其他节点有时间连接
    ros::spin();

    return 0;
}

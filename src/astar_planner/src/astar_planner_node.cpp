#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include "std_msgs/String.h"

// A*算法中使用的节点结构
struct Node {
    int x, y; // 栅格坐标
    double g_cost; // 从起点到此节点的实际代价
    double h_cost; // 从此节点到终点的启发式代价
    double f_cost() const { return g_cost + h_cost; }
    Node* parent;

    Node(int _x, int _y, double _g = 0, double _h = 0, Node* _p = nullptr)
        : x(_x), y(_y), g_cost(_g), h_cost(_h), parent(_p) {}

    // 用于优先队列的比较函数
    struct Compare {
        bool operator()(const Node* a, const Node* b) {
            return a->f_cost() > b->f_cost();
        }
    };
};

class AStarPlannerNode {
public:
    AStarPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), has_map_(false) {
           //ros::NodeHandle pnh("~");

        // 加载私有参数
        std::string pose_topic, goal_topic, waypoint_topic;
        pnh.param<std::string>("pose_topic", pose_topic, "drone_pose");
        pnh.param<std::string>("goal_topic", goal_topic, "llm_goal");
        pnh.param<std::string>("waypoint_topic", waypoint_topic, "drone_target");
        pnh.param<double>("waypoint_downsample_distance", waypoint_downsample_distance_, 7.0);

        node_namespace_ = nh_.getNamespace();
        if (node_namespace_ == "/") node_namespace_ = "[GLOBAL]"; // 美化一下全局命名空间
        
        // 读取全局参数时，需要一个根句柄
        ros::NodeHandle root_nh; 
        
        // 加载边界参数 (这些可以是全局的也可以是私有的，取决于您的设计)
        // 假设它们是全局的，以方便统一配置
        root_nh.param<double>("/boundary/min_x", boundary_min_x_, -45.0);
        root_nh.param<double>("/boundary/max_x", boundary_max_x_, 40.0);
        root_nh.param<double>("/boundary/min_y", boundary_min_y_, -40.0);
        root_nh.param<double>("/boundary/max_y", boundary_max_y_, 32.0);
        root_nh.param<double>("/boundary/min_z_height", boundary_min_z_height_, 0.5);
        root_nh.param<double>("/boundary/max_z_height", boundary_max_z_height_, 6.0);
        root_nh.param<double>("/boundary/safety_margin", safety_margin_, 1.5);
        root_nh.param<double>("/planning/inflation_radius", inflation_radius_meters_, 1.5); 
        
        // 订阅 (使用公共句柄 nh_，但传入的变量是相对名称)
        // 地图是全局的，所以硬编码为 "/map"
        map_sub_ = nh_.subscribe("/map", 1, &AStarPlannerNode::mapCallback, this);
        pose_sub_ = nh_.subscribe(pose_topic, 1, &AStarPlannerNode::poseCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic, 1, &AStarPlannerNode::goalCallback, this);

        // 发布 (同样使用公共句柄 nh_)
        waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(waypoint_topic, 10);
        plan_status_pub_ = nh_.advertise<std_msgs::String>("planning_status", 1, true); // true表示latching
        // 使用我们保存的命名空间来打印日志
        ROS_INFO_STREAM(node_namespace_ << " A* Planner Node is running.");
        ROS_INFO_STREAM(node_namespace_ << " Subscribing to pose on: " << pose_sub_.getTopic());
        ROS_INFO_STREAM(node_namespace_ << " Subscribing to goal on: " << goal_sub_.getTopic());
        ROS_INFO_STREAM(node_namespace_ << " Publishing waypoints on: " << waypoint_pub_.getTopic());
        ROS_INFO("A* Planner using inflation radius: %.2f meters.", inflation_radius_meters_);
    }

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        if (!has_map_) {
            map_ = *msg;
            has_map_ = true;
            //ROS_INFO("Map received! Size: %d x %d, Resolution: %.2f", map_.info.width, map_.info.height, map_.info.resolution);
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!has_map_) {
            ROS_WARN_STREAM(node_namespace_ << " A* Planner: No map available. Cannot plan path."); 
            return;
        }
        if (!current_pose_.header.frame_id.empty()) {
                    //检查目标点高度是否在允许范围内 ***
            double goal_z = msg->pose.position.z;
            if (goal_z < boundary_min_z_height_ || goal_z > boundary_max_z_height_) {
                     ROS_ERROR_STREAM(node_namespace_ << " A* Planner: Goal height " << std::fixed << std::setprecision(2) << goal_z
                       << " is outside the allowed flight envelope [" << boundary_min_z_height_
                       << ", " << boundary_max_z_height_ << "]. Aborting plan.");
                return;
            }
            ROS_INFO_STREAM(node_namespace_ << " New goal received. Planning from current pose to goal."); 
            planAndPublishFullPath(*msg);
        } else {
            ROS_WARN_STREAM(node_namespace_ << " A* Planner: Current pose not available. Cannot plan path.");
        }
    }

    void planAndPublishFullPath(const geometry_msgs::PoseStamped& goal_pose) {
        std_msgs::String status_msg;
        // 1. 规划路径 (和之前一样)
        Node start_node = worldToGrid(current_pose_.pose.position);
        Node goal_node = worldToGrid(goal_pose.pose.position);
        std::vector<Node*> full_path_nodes = findPath(start_node, goal_node);

        if (full_path_nodes.empty()) {
            ROS_ERROR_STREAM(node_namespace_ << " A* Planner: No path found to the goal!"); 
            status_msg.data = "failed";
            plan_status_pub_.publish(status_msg);
            return;
        }
        //ROS_INFO_STREAM(node_namespace_ << " A* Planner: Found a path with " << full_path_nodes.size() << " nodes."); 

        // 2. 对路径进行降采样，避免发布过多的密集航点
        std::vector<Node*> waypoints = downsamplePath(full_path_nodes, waypoint_downsample_distance_);
        //ROS_INFO("A* Planner: Downsampled path to %zu waypoints.", waypoints.size());
        ROS_INFO_STREAM(node_namespace_ << " A* Planner: Downsampled path to " << waypoints.size() << " waypoints.");
      // 在发布任何航点之前，先发布成功状态
    status_msg.data = "success";
    plan_status_pub_.publish(status_msg);      
        // 3. 循环遍历降采样后的航点，并逐一发布
        for (Node* wp_node : waypoints) {
            geometry_msgs::PoseStamped waypoint_pose;
            waypoint_pose.header.stamp = ros::Time::now();
            waypoint_pose.header.frame_id = "map"; // 使用统一的map frame
            waypoint_pose.pose.position = gridToWorld(*wp_node);
            waypoint_pose.pose.position.z = goal_pose.pose.position.z; 
            
            // 可以让无人机朝向下一个航点，或者简单地朝向最终目标
            waypoint_pose.pose.orientation = goal_pose.pose.orientation;

            waypoint_pub_.publish(waypoint_pose);
        ROS_INFO_STREAM(node_namespace_ << " A* Planner: Publishing waypoint ("
                        << std::fixed << std::setprecision(2) << waypoint_pose.pose.position.x << ", "
                        << std::fixed << std::setprecision(2) << waypoint_pose.pose.position.y << ", "
                        << std::fixed << std::setprecision(2) << waypoint_pose.pose.position.z << ")");
            
            ros::Duration(0.01).sleep(); // 加一个小延时，给ROS网络一点缓冲时间
        }
        ROS_INFO_STREAM(node_namespace_ << " A* Planner: All " << waypoints.size() << " waypoints have been published.");
    }
    
    // --- A* 核心实现 ---
    std::vector<Node*> findPath(const Node& start, const Node& goal) {
        std::priority_queue<Node*, std::vector<Node*>, Node::Compare> open_list;
        std::vector<bool> closed_list(map_.info.width * map_.info.height, false);
        std::vector<Node*> all_nodes;

        Node* start_node = new Node(start.x, start.y, 0, heuristic(start, goal));
        all_nodes.push_back(start_node);
        open_list.push(start_node);

        int debug_counter = 0; // [诊断新增] 防止日志刷屏

        while (!open_list.empty()) {
            Node* current = open_list.top();
            open_list.pop();

            // [诊断新增] 打印当前正在扩展的节点信息
            if(debug_counter < 10) { // 只打印前10个步骤，避免刷屏
                //ROS_INFO("Expanding node (%d, %d), f_cost: %.2f (g:%.2f, h:%.2f)", 
                //         current->x, current->y, current->f_cost(), current->g_cost, current->h_cost);
                debug_counter++;
            }

            if (current->x == goal.x && current->y == goal.y) {
                std::vector<Node*> path;
                while (current != nullptr) {
                    path.push_back(current);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            int current_idx = gridToIndex(current->x, current->y);
            if(closed_list[current_idx]) continue;
            closed_list[current_idx] = true;

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = current->x + dx;
                    int ny = current->y + dy;
                    if (isValid(nx, ny) && !isOccupied(nx, ny) && isWithinBoundaries(nx, ny) && !closed_list[gridToIndex(nx,ny)]) {
                        double move_cost = (dx == 0 || dy == 0) ? 1.0 : M_SQRT2;
                        double new_g_cost = current->g_cost + move_cost;
                        Node* neighbor = new Node(nx, ny, new_g_cost, heuristic({nx, ny}, goal), current);
                        all_nodes.push_back(neighbor);
                        open_list.push(neighbor);
                    }
                }
            }
        }
        
        // [诊断新增] 如果循环结束仍未找到路径，明确指出原因
       // ROS_WARN("A* search failed: Open list became empty before reaching the goal. Search space exhausted.");
        
        for(auto n : all_nodes) delete n;
        return {};
}
    // --- 辅助函数 ---
    std::vector<Node*> downsamplePath(const std::vector<Node*>& path, double min_distance) {
        if (path.size() < 2) {
            return path;
        }

        std::vector<Node*> downsampled_path;
        downsampled_path.push_back(path.front()); // 总是包含起点

        double distance_since_last_wp = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            distance_since_last_wp += heuristic(*path[i-1], *path[i]) * map_.info.resolution;

            // 如果距离足够远，或者这是最后一个点
            if (distance_since_last_wp >= min_distance || i == path.size() - 1) {
                downsampled_path.push_back(path[i]);
                distance_since_last_wp = 0.0; // 重置距离计数器
            }
        }
        return downsampled_path;
    }
    Node worldToGrid(const geometry_msgs::Point& p) {
        int gx = static_cast<int>((p.x - map_.info.origin.position.x) / map_.info.resolution);
        int gy = static_cast<int>((p.y - map_.info.origin.position.y) / map_.info.resolution);
        return Node(gx, gy);
    }

    geometry_msgs::Point gridToWorld(const Node& n) {
        geometry_msgs::Point p;
        p.x = n.x * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2.0;
        p.y = n.y * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2.0;
        return p;
    }
    
    int gridToIndex(int x, int y) {
        return y * map_.info.width + x;
    }

    double heuristic(const Node& a, const Node& b) {
        // 欧几里得距离
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    bool isValid(int x, int y) {
        return x >= 0 && x < map_.info.width && y >= 0 && y < map_.info.height;
    }

    bool isOccupied(int x, int y) {
        // 1. 将膨胀半径从米转换为栅格单位
        // ceil() 确保我们至少检查1个栅格，即使半径很小
        int inflation_radius_cells = static_cast<int>(ceil(inflation_radius_meters_ / map_.info.resolution));

        // 2. 检查以 (x, y) 为中心的邻域
        // 循环遍历一个 (2*radius+1) x (2*radius+1) 的正方形区域
        for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
            for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                int check_x = x + dx;
                int check_y = y + dy;

                // a. 确保检查的邻居点在地图范围内
                if (!isValid(check_x, check_y)) {
                    // 如果邻居在地图外，我们视其为障碍物以避免规划到地图边缘
                    return true; 
                }

                // b. 检查邻居点是否被占用
                // 占用栅格的值通常 > 50 (或者 > 0，取决于您的地图)
                if (map_.data[gridToIndex(check_x, check_y)] > 50) {
                    // 只要邻域内有一个点是障碍物，我们就认为当前点 (x, y) 是“危险”的，不可通行
                    return true;
                }
            }
        }

        // 3. 如果检查完整个邻域都没有发现障碍物，那么这个点是安全的
        return false;
    }
        //检查一个栅格坐标对应的世界坐标是否在安全边界内 ***
    bool isWithinBoundaries(int grid_x, int grid_y) {
        geometry_msgs::Point world_point = gridToWorld({grid_x, grid_y});
        
        if (world_point.x < boundary_min_x_ + safety_margin_ ||
            world_point.x > boundary_max_x_ - safety_margin_ ||
            world_point.y < boundary_min_y_ + safety_margin_ ||
            world_point.y > boundary_max_y_ - safety_margin_) {
            return false; // 在边界之外
        }
        return true; // 在边界之内
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_, pose_sub_, goal_sub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher plan_status_pub_;
    nav_msgs::OccupancyGrid map_;
    geometry_msgs::PoseStamped current_pose_;
    bool has_map_;
    double waypoint_downsample_distance_; 
        //边界和安全距离的成员变量 ***
    double boundary_min_x_, boundary_max_x_;
    double boundary_min_y_, boundary_max_y_;
    double boundary_min_z_height_, boundary_max_z_height_;
    double safety_margin_;
    double inflation_radius_meters_;
    std::string node_namespace_; // <--- 新增一个成员变量来存储命名空间
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner_node");
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); // 创建一个私有 NodeHandle，它的命名空间将是 /V_UAV_X/astar_planner_node
    AStarPlannerNode planner(nh, pnh); // 将这个私有句柄传入
    ros::spin();
    return 0;
}

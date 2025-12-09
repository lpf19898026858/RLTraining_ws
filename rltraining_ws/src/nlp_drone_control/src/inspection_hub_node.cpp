#include <ros/ros.h>
#include "nlp_drone_control/CaptureAndSave.h" // 新的服务头文件
#include "vlm_service/SaveImage.h"
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

class InspectionHubNode {
public:
    InspectionHubNode() {
        ros::NodeHandle nh;
        
        save_image_client_ = nh.serviceClient<vlm_service::SaveImage>("/vision_service/save_image");
        
        ROS_INFO("Waiting for VLM services (/vision_service/get_image, /vision_service/save_image)...");
        get_image_client_.waitForExistence();
        save_image_client_.waitForExistence();
        ROS_INFO("VLM services are available.");
        
        // 提供给Unity的服务
        capture_save_server_ = nh.advertiseService("/inspection_hub/capture_and_save", &InspectionHubNode::handleCaptureAndSave, this);
        
        // 在节点启动时创建一次带时间戳的文件夹路径
        generateTimestampFolder();
        
        ROS_INFO_STREAM("Inspection Hub Node is ready. Providing '/inspection_hub/capture_and_save' service. Images will be saved in: " << save_directory_);
    }

private:
    ros::ServiceServer capture_save_server_;
    ros::ServiceClient get_image_client_;
    ros::ServiceClient save_image_client_;
    
    std::string save_directory_;

    void generateTimestampFolder() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        std::string timestamp_folder = ss.str();
        
        // 设置一个基础路径，可以从参数服务器获取
        std::string base_save_path = "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/images";
        save_directory_ = base_save_path + "/" + timestamp_folder;
    }

    bool handleCaptureAndSave(nlp_drone_control::CaptureAndSave::Request &req,
                              nlp_drone_control::CaptureAndSave::Response &res)
    {
        ROS_INFO_STREAM("Received request from Unity to capture and save: " << req.file_name);

        // 步骤 2: 调用 save_image 服务
        vlm_service::SaveImage save_img_srv;
        save_img_srv.request.directory_path = save_directory_;
        save_img_srv.request.file_name = req.file_name;
        
        if (!save_image_client_.call(save_img_srv) || !save_img_srv.response.success) {
            ROS_ERROR_STREAM("Failed to save image: " << req.file_name);
            res.success = false;
            res.message = "Failed to save image via VLM service.";
            return true;
        }
        
        ROS_INFO_STREAM("Successfully captured and saved " << req.file_name);
        res.success = true;
        res.message = "Image saved.";
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "inspection_hub_node");
    InspectionHubNode node;
    ros::spin();
    return 0;
}

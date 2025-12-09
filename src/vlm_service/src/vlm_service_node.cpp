#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nlohmann/json.hpp>
#include "vlm_service/DescribeScene.h"
#include "vlm_service/CaptureImage.h"  // <<< 新增：用于同步拍照的服务
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <string>
#include <vector>
#include <mutex>
#include <cppcodec/base64_rfc4648.hpp>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <map> 
#include <boost/bind.hpp> 

using json = nlohmann::json;

const std::string VLM_API_ENDPOINT = "https://api.zhizengzeng.com/v1/chat/completions";

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

class VlmServiceNode {
public:
    // ★★★ 构造函数修改 ★★★
    VlmServiceNode(ros::NodeHandle& nh) : nh_(nh) {
        ros::NodeHandle pnh("~"); // 私有句柄，用于读取本节点的配置

        // 从全局参数服务器读取API Key
        nh_.getParam("/doubao_api_key", api_key_);
        if (api_key_.empty()) {
            ROS_FATAL("Global parameter '/doubao_api_key' not found! Shutting down.");
            ros::shutdown();
            return;
        }

        // 从私有参数服务器读取需要订阅的摄像头话题列表
        std::vector<std::string> camera_topics;
        pnh.param("camera_topics", camera_topics, std::vector<std::string>());
        
        if (camera_topics.empty()) {
            ROS_WARN("No 'camera_topics' specified in private params. VLM service will not receive any images.");
        }

        // 为列表中的每个话题动态创建订阅者
        for (const std::string& topic : camera_topics) {
            image_subs_.push_back(
                nh_.subscribe<sensor_msgs::Image>(
                    topic, 
                    1, 
                    boost::bind(&VlmServiceNode::imageCallback, this, _1, topic) // 使用boost::bind传递topic名
                )
            );
            ROS_INFO_STREAM("VLM Node subscribing to camera topic: " << topic);
        }

        // 发布服务 (使用公共句柄)
        describe_service_ = nh_.advertiseService("/vision_service/describe_scene", &VlmServiceNode::handleDescribeScene, this);
        capture_image_service_ = nh_.advertiseService("/vision_service/capture_image", &VlmServiceNode::handleCaptureImage, this);

        ROS_INFO("VLM Service Node is ready for concurrent requests.");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer describe_service_;
    ros::ServiceServer capture_image_service_; 
    std::string api_key_;
    
    // ★★★ 数据结构修改 ★★★
    // 使用map来存储来自不同话题的最新图像
    std::map<std::string, sensor_msgs::ImageConstPtr> latest_images_map_;
    std::mutex image_map_mutex_; // 一个互斥锁保护整个map
    std::vector<ros::Subscriber> image_subs_; // 存储订阅者对象以防被销毁

    // ★★★ imageCallback 修改 ★★★
    // 回调函数现在接收一个额外的参数，即消息来源的话题名
    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic) {
        std::lock_guard<std::mutex> lock(image_map_mutex_);
        latest_images_map_[topic] = msg; // 更新map
    }

    bool handleDescribeScene(vlm_service::DescribeScene::Request &req,
                             vlm_service::DescribeScene::Response &res) {
        ROS_INFO_STREAM("Received describe_scene request with " << req.images.size() << " images.");

        if (req.images.empty()) {
            ROS_WARN("DescribeScene called with zero images.");
            res.success = false;
            res.description = "Error: No images provided for description.";
            return true;
        }

        try {
            std::vector<std::string> base64_images;

            for (const auto& ros_image : req.images) {
                cv_bridge::CvImagePtr cv_ptr;
                try {
                    sensor_msgs::ImageConstPtr image_ptr(new sensor_msgs::Image(ros_image));
                    cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
            // <<< 在这里添加垂直翻转 >>>
            cv::Mat flipped_image;
            cv::flip(cv_ptr->image, flipped_image, 0); // 0 表示沿X轴翻转 (垂直翻转)
            cv_ptr->image = flipped_image; // 用翻转后的图像替换原来的 
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    res.success = false;
                    res.description = "Error converting one of the images.";
                    return true;
                }

                std::vector<unsigned char> image_data;
                if (!cv::imencode(".jpg", cv_ptr->image, image_data, {cv::IMWRITE_JPEG_QUALITY, 85})) {
                    ROS_ERROR("Failed to encode one of the images to JPG.");
                    res.success = false;
                    res.description = "Error encoding an image for VLM.";
                    return true;
                }

                if (image_data.empty()) {
                    ROS_ERROR("Encoded image data for one image is EMPTY!");
                    res.success = false;
                    res.description = "Error: Encoded image data is empty.";
                    return true;
                }
                base64_images.push_back(cppcodec::base64_rfc4648::encode(image_data));
            }

            ROS_INFO_STREAM("Successfully encoded " << base64_images.size() << " images to Base64.");

                std::string english_prompt = req.prompt + "\n\nPlease provide your response in English.";
    std::string vlm_response = callDoubaoVLMAPI(base64_images, english_prompt);
            
            if (vlm_response.empty()) {
                ROS_ERROR("Doubao-VLM API call failed or returned empty response.");
                res.success = false;
                res.description = "Error: Failed to get description from Doubao-VLM.";
                return true;
            }

            try {
                json response_json = json::parse(vlm_response);
                if (response_json.contains("choices") && response_json["choices"].is_array() &&
                    !response_json["choices"].empty() &&
                    response_json["choices"][0].contains("message") &&
                    response_json["choices"][0]["message"].contains("content")) {
                    res.description = response_json["choices"][0]["message"]["content"].get<std::string>();
                    res.success = true;
                    ROS_INFO("Doubao-VLM returned success. Description length: %zu", res.description.length());
                } else {
                    ROS_ERROR("Doubao-VLM response format error: %s", vlm_response.c_str());
                    res.success = false;
                    res.description = "Error: Doubao-VLM returned an unparseable response.";
                }
            } catch (const json::parse_error& e) {
                ROS_ERROR("JSON parse error: %s. Response: '%s'", e.what(), vlm_response.c_str());
                res.success = false;
                res.description = "Error: Doubao-VLM returned malformed JSON.";
            }

        } catch (const std::exception& e) {
            ROS_ERROR("Exception in describeSceneCallback: %s", e.what());
            res.success = false;
            res.description = std::string("An unexpected error occurred: ") + e.what();
        }
        return true;
    }

    // <<< 关键修正: 函数签名和内部逻辑已统一 >>>
    std::string callDoubaoVLMAPI(const std::vector<std::string>& base64_images, const std::string& prompt) {
        CURL *curl;
        CURLcode res_code;
        std::string read_buffer;

        curl = curl_easy_init();
        if (curl) {
            // 构建请求体
            json content_array = json::array();
            
            // 1. 添加文本提示
            content_array.push_back({
                {"type", "text"},
                {"text", prompt}
            });

            // 2. 循环添加所有图片
            for (const auto& base64_image_str : base64_images) { // 使用正确的变量名
                content_array.push_back({
                    {"type", "image_url"},
                    {"image_url", {
                        {"url", "data:image/jpeg;base64," + base64_image_str}
                    }}
                });
            }
            
            // 旧的、冲突的逻辑已被移除

            json request_body = {
                {"model", "doubao-1-5-vision-pro-250328"},
                {"messages", {{
                    {"role", "user"},
                    {"content", content_array}
                }}},
                {"stream", false},
                {"max_tokens", 2048}
            };
            std::string request_body_str = request_body.dump();

            struct curl_slist *headers = NULL;
            // ... (cURL 调用逻辑不变)
            headers = curl_slist_append(headers, "Content-Type: application/json");
            headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key_).c_str());

            curl_easy_setopt(curl, CURLOPT_URL, VLM_API_ENDPOINT.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body_str.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, request_body_str.length());

        const char* proxy_url = "http://127.0.0.1:7897";
        curl_easy_setopt(curl, CURLOPT_PROXY, proxy_url);

            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &read_buffer);

            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

            curl_easy_setopt(curl, CURLOPT_VERBOSE, 0L);

            ROS_INFO("Calling Doubao-VLM API via proxy %s...", proxy_url); // 修改日志方便确认
            res_code = curl_easy_perform(curl);

            if (res_code != CURLE_OK) {
                ROS_ERROR("curl_easy_perform() failed: %s", curl_easy_strerror(res_code));
                read_buffer.clear();
            } else {
                long http_code = 0;
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
                if (http_code != 200) {
                    ROS_ERROR("Doubao-VLM API returned HTTP %ld. Response: '%s'", http_code, read_buffer.c_str());
                    read_buffer.clear();
                }
            }
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        } else {
            ROS_ERROR("Failed to initialize cURL.");
        }
        return read_buffer;
    }

    // 不再从单一的 latest_image_ 获取，而是从 map 中按需查找
    bool handleCaptureImage(vlm_service::CaptureImage::Request &req,
                            vlm_service::CaptureImage::Response &res)
    {
        ROS_INFO_STREAM("Received capture request for topic [" << req.image_topic << "] to file " << req.file_name);
        
	    sensor_msgs::ImageConstPtr current_image;
        {
            std::lock_guard<std::mutex> lock(image_map_mutex_);
            // 在map中查找请求的话题
            if (latest_images_map_.find(req.image_topic) == latest_images_map_.end()) {
                ROS_WARN_STREAM("No image received yet on topic '" << req.image_topic << "'. Cannot capture.");
                res.success = false;
                res.message = "Error: No image data available for topic " + req.image_topic;
                return true;
            }
            current_image = latest_images_map_[req.image_topic];
        }

        try {
            std::filesystem::path dir(req.directory_path);
            if (!std::filesystem::exists(dir)) {
                if (!std::filesystem::create_directories(dir)) {
                    ROS_ERROR_STREAM("Failed to create directory: " << req.directory_path);
                    res.success = false;
                    res.message = "Error: Could not create directory for saving image.";
                    res.file_path = "";
                    return true;
                }
            }

            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(current_image, sensor_msgs::image_encodings::BGR8);
            
            cv::Mat flipped_image;
            cv::flip(cv_ptr->image, flipped_image, 0);
            
            std::string full_path = req.directory_path + "/" + req.file_name;
            if (cv::imwrite(full_path, flipped_image)) {
                ROS_INFO_STREAM("Successfully captured and saved flipped image to: " << full_path);
                res.success = true;
                res.message = "Image captured and saved successfully.";
                res.file_path = full_path; //返回完整路径
                res.captured_image = *current_image; // 将原始图像数据放入响应中
            } else {
                ROS_ERROR_STREAM("Failed to write image to: " << full_path);
                res.success = false;
                res.message = "Error: Failed to write image to disk.";
                res.file_path = "";
            }

        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception during capture: %s", e.what());
            res.success = false;
            res.message = "Error converting image format.";
            res.file_path = "";
            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in handleCaptureImage: %s", e.what());
            res.success = false;
            res.message = std::string("An unexpected error occurred: ") + e.what();
            res.file_path = "";
        }

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vlm_service_node");
    ros::NodeHandle nh;
    VlmServiceNode node(nh); // 将句柄传入构造函数

    // 使用多线程Spinner来并发处理服务请求
    // 这里的4表示最多可以同时处理4个请求
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}

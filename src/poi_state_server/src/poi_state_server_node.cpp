#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <map>

#include "poi_state_server/GetPOIInfo.h"
#include "poi_state_server/ListPOIs.h"
#include "poi_state_server/POIInfo.h"
#include "poi_state_server/NamedPoint.h"

struct NamedPointInternal {
  std::string name;
  geometry_msgs::Point point;
};

struct Boundary {
  geometry_msgs::Point min;
  geometry_msgs::Point max;
  bool valid = false;
};

struct POIData {
  std::string name;
  std::string type;
  std::string description;
  geometry_msgs::Point position;
  std::vector<NamedPointInternal> candidate_points;
  Boundary simplified_boundary;
};

std::map<std::string, POIData> poi_map;

// ================== Helpers ==================
poi_state_server::POIInfo toMsg(const POIData &data) {
  poi_state_server::POIInfo msg;
  msg.name = data.name;
  msg.type = data.type;
  msg.description = data.description;
  msg.position = data.position;
  msg.has_boundary = data.simplified_boundary.valid;
  msg.boundary_min = data.simplified_boundary.min;
  msg.boundary_max = data.simplified_boundary.max;

  for (const auto &cand : data.candidate_points) {
    poi_state_server::NamedPoint np;
    np.name = cand.name;
    np.point = cand.point;
    msg.candidate_points.push_back(np);
  }
  return msg;
}

// ================== Service Handlers ==================

bool getPOIInfo(poi_state_server::GetPOIInfo::Request &req,
                poi_state_server::GetPOIInfo::Response &res) {
  auto it = poi_map.find(req.name);
  if (it == poi_map.end()) {
    res.success = false;
    res.message = "POI " + req.name + " not found.";
    return true;
  }

  res.success = true;
  res.message = "OK";
  res.info = toMsg(it->second);
  return true;
}

bool listPOIs(poi_state_server::ListPOIs::Request &,
              poi_state_server::ListPOIs::Response &res) {
  res.success = true;
  res.message = "OK";

  for (const auto &kv : poi_map) {
    res.pois.push_back(toMsg(kv.second));
  }
  return true;
}

// ================== Main ==================

int main(int argc, char **argv) {
  ros::init(argc, argv, "poi_state_server");
  ros::NodeHandle nh("~");

  std::string poi_file;
  nh.param<std::string>("poi_file", poi_file, "points_of_interest.yaml");

  try {
    YAML::Node root = YAML::LoadFile(poi_file);
    for (auto poi : root["points_of_interest"]) {
      POIData data;
      data.name = poi["name"].as<std::string>();
      data.type = poi["type"].as<std::string>();
      data.description = poi["description"].as<std::string>();

      // position
      data.position.x = poi["position"]["x"].as<double>();
      data.position.y = poi["position"]["y"].as<double>();
      data.position.z = poi["position"]["z"].as<double>();

      // candidate_interaction_points
      if (poi["candidate_interaction_points"]) {
        for (auto cand : poi["candidate_interaction_points"]) {
          NamedPointInternal np;
          np.name = cand["name"].as<std::string>();
          np.point.x = cand["point"]["x"].as<double>();
          np.point.y = cand["point"]["y"].as<double>();
          np.point.z = cand["point"]["z"].as<double>();
          data.candidate_points.push_back(np);
        }
      }

      // simplified_boundary
      if (poi["simplified_boundary"]) {
        data.simplified_boundary.valid = true;
        data.simplified_boundary.min.x = poi["simplified_boundary"]["min"]["x"].as<double>();
        data.simplified_boundary.min.y = poi["simplified_boundary"]["min"]["y"].as<double>();
        data.simplified_boundary.min.z = poi["simplified_boundary"]["min"]["z"].as<double>();
        data.simplified_boundary.max.x = poi["simplified_boundary"]["max"]["x"].as<double>();
        data.simplified_boundary.max.y = poi["simplified_boundary"]["max"]["y"].as<double>();
        data.simplified_boundary.max.z = poi["simplified_boundary"]["max"]["z"].as<double>();
      }

      poi_map[data.name] = data;
    }
    ROS_INFO("Loaded %zu POIs from %s", poi_map.size(), poi_file.c_str());
  } catch (const std::exception &e) {
    ROS_FATAL("Failed to load POI yaml: %s", e.what());
    return 1;
  }

  ros::ServiceServer srv1 = nh.advertiseService("get_poi_info", getPOIInfo);
  ros::ServiceServer srv2 = nh.advertiseService("list_pois", listPOIs);

  ros::spin();
  return 0;
}


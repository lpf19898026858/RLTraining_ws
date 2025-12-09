#include "nlp_drone_control/task_executor_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_executor_node");
  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner;
  TaskExecutorNode node(nh);
  spinner.spin();
  return 0;
}


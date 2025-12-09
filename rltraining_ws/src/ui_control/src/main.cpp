/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include <ros/ros.h> // <--- 引入 ROS 头文件
#include "../include/ui_control/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** ROS Initialisation
    **********************/
    // 1. 初始化 ROS 节点，这是第一步！
    ros::init(argc, argv, "ui_control_node");

    // 2. 创建一个 NodeHandle，它将被传递给 GUI
    ros::NodeHandle nh("~"); // 使用私有命名空间 "~" 是个好习惯

    /*********************
    ** Qt Application
    **********************/
    QApplication app(argc, argv);

    // 3. 创建主窗口，并将 NodeHandle 传递给它
    ui_control::MainWindow w(nh); // <--- 注意这里的变化
    w.show();

    // 4. 设置关闭逻辑
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    // 5. 启动 Qt 事件循环
    int result = app.exec();

    return result;
}
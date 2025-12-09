/*
© Guangdong University of Technology,
© The Laboratory of Intelligent Decision and Cooperative Control,
© 2021-2022,
© Author: Yuanlin Yang (yongwang0808@163.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include "v_uav_local_network.h"
#include "ui_v_uav_local_network.h"

bool v_uav_local_network::set_object_id(int id)
{
    _object_id = id;
    return true;
}
void v_uav_local_network::camera_sub_cb(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  _camera_data = cv_ptr_compressed->image;
}

void v_uav_local_network::camera_request_sub_cb(const std_msgs::Bool::ConstPtr &msg)
{
  _camera_request=msg.get()->data;
  if(_camera_request)
  {
     if(!_camera_data.empty())
     {
       QImage v_uav_0_Img;
       cv::Mat v_uav_0_Rgb;
       if (_camera_data.channels() == 3)//RGB Img
       {
         cv::cvtColor(_camera_data, v_uav_0_Rgb, CV_BGR2RGB);//颜色空间转换
         v_uav_0_Img = QImage((const uchar*)(v_uav_0_Rgb.data), v_uav_0_Rgb.cols, v_uav_0_Rgb.rows, v_uav_0_Rgb.cols * v_uav_0_Rgb.channels(), QImage::Format_RGB888);
       }
       else//Gray Img
       {
         v_uav_0_Img = QImage((const uchar*)(_camera_data.data),_camera_data.cols, _camera_data.rows, _camera_data.cols*_camera_data.channels(), QImage::Format_Indexed8);
       }
       ui->v_uav_0_camera_image->setPixmap(QPixmap::fromImage(v_uav_0_Img));
     }
  }
  else
  {
  	    // 获取显示区域的大小
    int width = ui->v_uav_0_camera_image->width();
    int height = ui->v_uav_0_camera_image->height();

    // 创建一个黑色的 QImage
    QImage blackImage(width, height, QImage::Format_RGB888);
    blackImage.fill(Qt::black);

    // 使用 QPainter 在图像中间绘制文字 "Close"
    QPainter painter(&blackImage);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 24));
    QRect textRect(0, 0, width, height);
    painter.drawText(textRect, Qt::AlignCenter, "Close");
    painter.end();

    // 显示带有文字的黑色图像
    ui->v_uav_0_camera_image->setPixmap(QPixmap::fromImage(blackImage));
  	//QImage v_uav_0_Img;
  	//v_uav_0_Img = QImage((const uchar*)(_camera_data.data),_camera_data.cols, _camera_data.rows, _camera_data.cols*_camera_data.channels(), QImage::Format_Indexed8);
        //ui->v_uav_0_camera_image->setPixmap(QPixmap::fromImage(v_uav_0_Img));
  }
}

void v_uav_local_network::current_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  _current_pose.pose = msg.get()->pose;
  _current_move_track_msg.header.frame_id="v_uav_local";
  geometry_msgs::PoseStamped this_pose_stamped;
  this_pose_stamped.pose.position.x = msg.get()->pose.position.x;
  this_pose_stamped.pose.position.y = msg.get()->pose.position.y;
  this_pose_stamped.pose.position.z = msg.get()->pose.position.z;
  this_pose_stamped.header.frame_id="v_uav_local";
  _current_move_track_msg.poses.push_back(this_pose_stamped);
  _movement_track_pub.publish(_current_move_track_msg);
}

bool v_uav_local_network::set_network_name(std::string networkName)
{
_network_name = networkName;
return true;
}

bool v_uav_local_network::set_object_name(std::string objectName)
{
    _object_name = objectName;
    return true;
}

void v_uav_local_network::init_ros()
{
    ros::NodeHandle n("~");
    std::string tempString;

    n.getParam("mission_start_pub_topic",tempString);
    _mission_start_pub_topic = "/"+_object_name+tempString;

    n.getParam("current_pose_sub_topic",tempString);
    _current_pose_sub_topic = "/"+_object_name+"/"+_network_name+tempString;

    n.getParam("current_camera_sub_topic",tempString);
    _current_camera_sub_topic = tempString;

    n.getParam("movement_track_pub_topic",tempString);
    _movement_track_pub_topic = "/"+_object_name+"/"+_network_name+tempString;

    n.getParam("request_camera_data_pub_topic",tempString);
    _request_camera_data_pub_topic =  "/"+_object_name+"/"+_network_name+tempString;

    n.getParam("target_movement_track_pub_topic",tempString);
    _target_movement_track_pub_topic = "/"+_object_name+"/"+_network_name+tempString;
    
    n.getParam("request_camera_data_sub_topic",tempString);
    _request_camera_data_sub_topic =  "/"+_object_name+"/"+_network_name+tempString;

    n.getParam("nlp_command_pub_topic",tempString);
    _nlp_command_pub_topic = tempString;

    n.getParam("nlp_feedback_sub_topic",tempString);
    _nlp_feedback_sub_topic =  tempString;

    n.getParam("nlp_stop_pub_topic", tempString);
    _nlp_stop_pub_topic = tempString;     
    n.getParam("nlp_rereasoning_pub_topic", tempString);
    _nlp_rereasoning_pub_topic = tempString;     
    n.getParam("nlp_run_pub_topic", tempString);
    _nlp_run_pub_topic = tempString;     
    n.getParam("nlp_reason_sub_topic", tempString);
    _nlp_reason_sub_topic = tempString; 

     _nlp_command_pub=n.advertise<std_msgs::String>(_nlp_command_pub_topic,10);
     _mission_start_pub = n.advertise<dt_message_package::mission_msg>(_mission_start_pub_topic,1);
     _nlp_feedback_sub=n.subscribe(_nlp_feedback_sub_topic,100,&v_uav_local_network::feedbackCallback,this);
     _current_pose_sub = n.subscribe(_current_pose_sub_topic,1,&v_uav_local_network::current_position_sub_cb,this);
     _request_camera_data_sub = n.subscribe(_request_camera_data_sub_topic,1,&v_uav_local_network::camera_request_sub_cb,this);
     _movement_track_pub = n.advertise<nav_msgs::Path>(_movement_track_pub_topic,1);
     _current_camera_sub = n.subscribe(_current_camera_sub_topic,1,&v_uav_local_network::camera_sub_cb,this);
     _request_camera_data_pub = n.advertise<std_msgs::Bool>(_request_camera_data_pub_topic,1);
     _target_movement_track_pub = n.advertise<nav_msgs::Path>(_target_movement_track_pub_topic,1);
     _nlp_stop_pub = n.advertise<std_msgs::Empty>(_nlp_stop_pub_topic, 1);
    _nlp_reasoning_sub=n.subscribe(_nlp_reason_sub_topic, 100, &v_uav_local_network::reasoningCallback, this);
    _nlp_rereasoning_pub = n.advertise<std_msgs::Empty>(_nlp_rereasoning_pub_topic, 1);
    _nlp_run_pub = n.advertise<std_msgs::Empty>(_nlp_run_pub_topic, 1);
    
     std::string label_uav = "V_UAV_"+std::to_string(_object_id);
     QString label_uav_qt = QString::fromLocal8Bit(label_uav.data());
     //ui->v_uav_0_label_3->setText(label_uav_qt);
     QPalette pe;
     pe.setColor(QPalette::WindowText,Qt::green);
     //ui->v_uav_0_label_3->setPalette(pe);

}

v_uav_local_network::v_uav_local_network(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::v_uav_local_network),
  m_isStreamingInProgress(false) // 在构造函数中初始化状态变量
{
  ui->setupUi(this);
  m_nTimerID = this->startTimer(500);

  _current_pose.pose.position.x = 0.0;
  _current_pose.pose.position.y = 0.0;
  _current_pose.pose.position.z = 0.0;
  // 连接信号和槽！
  connect(this, &v_uav_local_network::newFeedbackReceived, 
            this, &v_uav_local_network::updateFeedbackDisplay);
  connect(this, &v_uav_local_network::newReasoningReceived, 
            this, &v_uav_local_network::updateReasoningDisplay);
  // 设置只读和换行?
  ui->feedback_text_edit->setReadOnly(true);
  ui->feedback_text_edit->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  ui->reasoning_text_edit->setReadOnly(true);
  ui->reasoning_text_edit->setLineWrapMode(QPlainTextEdit::WidgetWidth);

}

v_uav_local_network::~v_uav_local_network()
{
  this->killTimer(m_nTimerID);
  delete ui;
}

void v_uav_local_network::on_v_uav_0_mission_start_button_clicked()
{
    bool is_mission = true;
    uint32_t mission_id = ui->v_uav_0_mission_id_edit->text().toInt();
    dt_message_package::mission_msg mission_msg;
    mission_msg.mission_id = mission_id;
    mission_msg.mission_state = is_mission;
   _mission_start_pub.publish(mission_msg);
}

void v_uav_local_network::on_v_uav_0_camera_open_button_clicked()
{
    std_msgs::Bool request_camera_data_msg;
    request_camera_data_msg.data = true;
    _request_camera_data_pub.publish(request_camera_data_msg);
}

void v_uav_local_network::on_v_uav_0_camera_close_button_clicked()
{
    std_msgs::Bool request_camera_data_msg;
    request_camera_data_msg.data = false;
    _request_camera_data_pub.publish(request_camera_data_msg);
}
void v_uav_local_network::on_v_uav_0_send_command_button_clicked()
{
  ROS_INFO("Click_send_command_button!");
  std_msgs::String ref_msg;
  ref_msg.data = ui->v_uav_0_ref_nlp_command->text().toStdString();
  _nlp_command_pub.publish(ref_msg);
}
void v_uav_local_network::feedbackCallback(const std_msgs::String::ConstPtr& msg)
{
//ROS_INFO_STREAM("Raw data in feedbackCallback: " << msg->data.c_str()); 
    // 从ROS回调更新Qt UI需要使用信号/槽机制，确保线程安全
    Q_EMIT newFeedbackReceived(QString::fromStdString(msg->data));
}
void v_uav_local_network::on_v_uav_0_stop_command_button_clicked()
{
    ROS_INFO("Stop button clicked! Publishing stop command.");
    
    // 创建一个空的 std_msgs::Empty 消息
    std_msgs::Empty stop_msg;
    
    // 发布消息到 /nlp_stop topic
    _nlp_stop_pub.publish(stop_msg);
}
void v_uav_local_network::on_v_uav_0_rereasoning_command_button_clicked()
{
    ROS_INFO("Rereasoning button clicked! Publishing rereasoning command.");
    
    std_msgs::Empty rereasoning_msg;
    
    // 发布消息到 /nlp_rereasoning_topic
    _nlp_rereasoning_pub.publish(rereasoning_msg);
}
void v_uav_local_network::on_v_uav_0_run_command_button_clicked()
{
    ROS_INFO("Run button clicked! Publishing run command.");
    
    // 创建一个空的 std_msgs::Empty 消息
    std_msgs::Empty run_msg;
    
    // 发布消息到 /nlp_stop topic
    _nlp_run_pub.publish(run_msg);
}

void v_uav_local_network::updateFeedbackDisplay(const QString &text)
{
    //ROS_INFO("Received text for display: %s", text.toStdString().c_str());
    // 标记1: 这是一条开始流式会话的指令吗？
    if (text == "LLM_STREAM_START") {
        m_isStreamingInProgress = true; // 准备好接收流式消息
        //if (!ui->feedback_text_edit->toPlainText().isEmpty()) {
        //    ui->feedback_text_edit->clear();
        //}
        return;
    }
    
    // 判断新消息是否是LLM的流式输出
    bool isStreamingNow = text.startsWith("LLM: ");

    if (isStreamingNow) {
    QString content = text.mid(5); // 去掉 "LLM: " 前缀
    	if (m_isStreamingInProgress) {
    	    if (!ui->feedback_text_edit->toPlainText().isEmpty()) {
                ui->feedback_text_edit->appendPlainText(""); 
            }
        // 这是流式输出的第一个片段
        // 使用 appendPlainText 来开始一个新行（段落）
        ui->feedback_text_edit->appendPlainText(content);
        m_isStreamingInProgress = false; // 标记流式输出已开始
    } else {
        // 这是流式输出的后续片段
        // 移动光标到末尾
        QTextCursor cursor = ui->feedback_text_edit->textCursor();
        cursor.movePosition(QTextCursor::End);
        ui->feedback_text_edit->setTextCursor(cursor);
        
        // 使用 insertPlainText 在当前行追加文本，不会自动换行
        ui->feedback_text_edit->insertPlainText(content);
    }
} else {
    // 这不是一条流式消息，或者标志着流式输出的结束
    m_isStreamingInProgress = false; // 重置标记

    // 直接追加这条完整的消息，并换行
    ui->feedback_text_edit->appendPlainText(text);
	}

// 同样，确保视图滚动到底部
	ui->feedback_text_edit->verticalScrollBar()->setValue(ui->feedback_text_edit->verticalScrollBar()->maximum());
}

// ++ 新增：实现新的回调函数和槽函数
void v_uav_local_network::reasoningCallback(const std_msgs::String::ConstPtr& msg)
{
    Q_EMIT newReasoningReceived(QString::fromStdString(msg->data));
}
void v_uav_local_network::updateReasoningDisplay(const QString &text)
{
    // 思考过程的显示逻辑可以简化
    // 移动光标到末尾
    QTextCursor cursor = ui->reasoning_text_edit->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->reasoning_text_edit->setTextCursor(cursor);
    
    // 插入文本
    ui->reasoning_text_edit->insertPlainText(text);
    //ui->reasoning_text_edit->insertPlainText("\n");
    
    // 滚动到底部
    ui->reasoning_text_edit->verticalScrollBar()->setValue(ui->reasoning_text_edit->verticalScrollBar()->maximum());
}

void v_uav_local_network::on_clear_reasoning_button_clicked()
{
    ui->reasoning_text_edit->clear();
}

void v_uav_local_network::on_clear_feedback_button_clicked()
{
    ui->feedback_text_edit->clear();
}

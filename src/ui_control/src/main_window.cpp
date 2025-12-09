
/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ui_control/main_window.hpp"
#include "log_in.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ui_control {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(ros::NodeHandle &nh, QWidget *parent)
  : QMainWindow(parent)
  , qnode(nh) // <--- 将 NodeHandle 传递给 QNode 的新构造函数
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  //  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
    ** Auto Start & Connection
    **********************/
  // 由于 ros::init 已经在 main 中完成，我们可以直接在这里初始化 qnode
  // 不再需要 Connect 按钮来触发 init
  
  // 直接初始化 qnode
  qnode.init(); 

  // 禁用连接相关的 UI 元素，因为已经自动连接了
  ui.button_connect->setEnabled(false);
  ui.line_edit_master->setReadOnly(true);
  ui.line_edit_host->setReadOnly(true);
  ui.line_edit_topic->setReadOnly(true);
  ui.checkbox_use_environment->setEnabled(false);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

/*void MainWindow::on_button_connect_clicked(bool check ) {
  if ( ui.checkbox_use_environment->isChecked() ) {
    if ( !qnode.init() ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                      ui.line_edit_host->text().toStdString()) ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
      ui.line_edit_topic->setReadOnly(true);
    }
  }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if ( state == 0 ) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
  //ui.line_edit_topic->setEnabled(enabled);
}
*/
/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  //ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "server_0");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url = settings.value("master_url",QString("http://172.17.0.2:11311/")).toString();
  QString host_url = settings.value("host_url", QString("172.17.0.2")).toString();
  //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  //ui.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if ( checked ) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
    //ui.line_edit_topic->setEnabled(false);
  }
}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "server_0");
  settings.setValue("master_url",ui.line_edit_master->text());
  settings.setValue("host_url",ui.line_edit_host->text());
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace server_0




void ui_control::MainWindow::on_enter_server_0_pushButton_clicked()
{
   log_in* log = new log_in;
   log->show();
}

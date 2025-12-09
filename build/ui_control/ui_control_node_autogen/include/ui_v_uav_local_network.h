/********************************************************************************
** Form generated from reading UI file 'v_uav_local_network.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_V_UAV_LOCAL_NETWORK_H
#define UI_V_UAV_LOCAL_NETWORK_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_v_uav_local_network
{
public:
    QWidget *centralwidget;
    QLabel *v_uav_0_camera_image;
    QPushButton *v_uav_0_init_button;
    QLabel *v_uav_0_label;
    QLabel *v_uav_0_current_position_label;
    QLineEdit *v_uav_0_mission_id_edit;
    QLabel *v_uav_0_mission_label;
    QPushButton *v_uav_0_camera_open_button;
    QPushButton *v_uav_0_mission_start_button;
    QLabel *v_uav_0_mission_id_label;
    QPushButton *v_uav_0_camera_close_button;
    QLineEdit *v_uav_0_ref_nlp_command;
    QPushButton *v_uav_0_send_command_button;
    QPlainTextEdit *feedback_text_edit;
    QLabel *v_uav_0_mission_id_label_2;
    QLabel *v_uav_0_mission_id_label_3;
    QLineEdit *v_uav_0_mission_id_edit_2;
    QLabel *v_uav_0_mission_id_label_4;
    QLineEdit *v_uav_0_mission_id_edit_3;
    QLineEdit *v_uav_0_mission_id_edit_4;
    QPushButton *v_uav_0_stop_command_button;
    QPlainTextEdit *reasoning_text_edit;
    QPushButton *clear_reasoning_button;
    QPushButton *clear_feedback_button;
    QPushButton *v_uav_0_rereasoning_command_button;
    QPushButton *v_uav_0_run_command_button;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *v_uav_local_network)
    {
        if (v_uav_local_network->objectName().isEmpty())
            v_uav_local_network->setObjectName(QString::fromUtf8("v_uav_local_network"));
        v_uav_local_network->resize(950, 763);
        centralwidget = new QWidget(v_uav_local_network);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        v_uav_0_camera_image = new QLabel(centralwidget);
        v_uav_0_camera_image->setObjectName(QString::fromUtf8("v_uav_0_camera_image"));
        v_uav_0_camera_image->setGeometry(QRect(70, 460, 391, 231));
        v_uav_0_camera_image->setFrameShape(QFrame::WinPanel);
        v_uav_0_init_button = new QPushButton(centralwidget);
        v_uav_0_init_button->setObjectName(QString::fromUtf8("v_uav_0_init_button"));
        v_uav_0_init_button->setGeometry(QRect(710, 560, 80, 22));
        v_uav_0_label = new QLabel(centralwidget);
        v_uav_0_label->setObjectName(QString::fromUtf8("v_uav_0_label"));
        v_uav_0_label->setGeometry(QRect(870, 50, 61, 51));
        v_uav_0_label->setTextFormat(Qt::RichText);
        v_uav_0_label->setPixmap(QPixmap(QString::fromUtf8(":/images/uav.jpeg")));
        v_uav_0_label->setScaledContents(true);
        v_uav_0_current_position_label = new QLabel(centralwidget);
        v_uav_0_current_position_label->setObjectName(QString::fromUtf8("v_uav_0_current_position_label"));
        v_uav_0_current_position_label->setGeometry(QRect(750, 140, 131, 31));
        v_uav_0_mission_id_edit = new QLineEdit(centralwidget);
        v_uav_0_mission_id_edit->setObjectName(QString::fromUtf8("v_uav_0_mission_id_edit"));
        v_uav_0_mission_id_edit->setGeometry(QRect(760, 520, 81, 22));
        v_uav_0_mission_label = new QLabel(centralwidget);
        v_uav_0_mission_label->setObjectName(QString::fromUtf8("v_uav_0_mission_label"));
        v_uav_0_mission_label->setGeometry(QRect(690, 480, 131, 31));
        v_uav_0_camera_open_button = new QPushButton(centralwidget);
        v_uav_0_camera_open_button->setObjectName(QString::fromUtf8("v_uav_0_camera_open_button"));
        v_uav_0_camera_open_button->setGeometry(QRect(500, 530, 80, 22));
        v_uav_0_mission_start_button = new QPushButton(centralwidget);
        v_uav_0_mission_start_button->setObjectName(QString::fromUtf8("v_uav_0_mission_start_button"));
        v_uav_0_mission_start_button->setGeometry(QRect(710, 600, 80, 22));
        v_uav_0_mission_id_label = new QLabel(centralwidget);
        v_uav_0_mission_id_label->setObjectName(QString::fromUtf8("v_uav_0_mission_id_label"));
        v_uav_0_mission_id_label->setGeometry(QRect(660, 520, 71, 20));
        v_uav_0_camera_close_button = new QPushButton(centralwidget);
        v_uav_0_camera_close_button->setObjectName(QString::fromUtf8("v_uav_0_camera_close_button"));
        v_uav_0_camera_close_button->setGeometry(QRect(500, 570, 80, 22));
        v_uav_0_ref_nlp_command = new QLineEdit(centralwidget);
        v_uav_0_ref_nlp_command->setObjectName(QString::fromUtf8("v_uav_0_ref_nlp_command"));
        v_uav_0_ref_nlp_command->setGeometry(QRect(40, 60, 531, 22));
        v_uav_0_send_command_button = new QPushButton(centralwidget);
        v_uav_0_send_command_button->setObjectName(QString::fromUtf8("v_uav_0_send_command_button"));
        v_uav_0_send_command_button->setGeometry(QRect(590, 60, 80, 22));
        feedback_text_edit = new QPlainTextEdit(centralwidget);
        feedback_text_edit->setObjectName(QString::fromUtf8("feedback_text_edit"));
        feedback_text_edit->setGeometry(QRect(380, 110, 301, 271));
        v_uav_0_mission_id_label_2 = new QLabel(centralwidget);
        v_uav_0_mission_id_label_2->setObjectName(QString::fromUtf8("v_uav_0_mission_id_label_2"));
        v_uav_0_mission_id_label_2->setGeometry(QRect(720, 190, 71, 20));
        v_uav_0_mission_id_label_3 = new QLabel(centralwidget);
        v_uav_0_mission_id_label_3->setObjectName(QString::fromUtf8("v_uav_0_mission_id_label_3"));
        v_uav_0_mission_id_label_3->setGeometry(QRect(720, 220, 71, 20));
        v_uav_0_mission_id_edit_2 = new QLineEdit(centralwidget);
        v_uav_0_mission_id_edit_2->setObjectName(QString::fromUtf8("v_uav_0_mission_id_edit_2"));
        v_uav_0_mission_id_edit_2->setGeometry(QRect(810, 190, 81, 22));
        v_uav_0_mission_id_label_4 = new QLabel(centralwidget);
        v_uav_0_mission_id_label_4->setObjectName(QString::fromUtf8("v_uav_0_mission_id_label_4"));
        v_uav_0_mission_id_label_4->setGeometry(QRect(720, 250, 71, 20));
        v_uav_0_mission_id_edit_3 = new QLineEdit(centralwidget);
        v_uav_0_mission_id_edit_3->setObjectName(QString::fromUtf8("v_uav_0_mission_id_edit_3"));
        v_uav_0_mission_id_edit_3->setGeometry(QRect(810, 220, 81, 22));
        v_uav_0_mission_id_edit_4 = new QLineEdit(centralwidget);
        v_uav_0_mission_id_edit_4->setObjectName(QString::fromUtf8("v_uav_0_mission_id_edit_4"));
        v_uav_0_mission_id_edit_4->setGeometry(QRect(810, 250, 81, 22));
        v_uav_0_stop_command_button = new QPushButton(centralwidget);
        v_uav_0_stop_command_button->setObjectName(QString::fromUtf8("v_uav_0_stop_command_button"));
        v_uav_0_stop_command_button->setGeometry(QRect(680, 60, 80, 22));
        reasoning_text_edit = new QPlainTextEdit(centralwidget);
        reasoning_text_edit->setObjectName(QString::fromUtf8("reasoning_text_edit"));
        reasoning_text_edit->setGeometry(QRect(40, 110, 321, 271));
        clear_reasoning_button = new QPushButton(centralwidget);
        clear_reasoning_button->setObjectName(QString::fromUtf8("clear_reasoning_button"));
        clear_reasoning_button->setGeometry(QRect(260, 390, 80, 22));
        clear_feedback_button = new QPushButton(centralwidget);
        clear_feedback_button->setObjectName(QString::fromUtf8("clear_feedback_button"));
        clear_feedback_button->setGeometry(QRect(450, 390, 80, 22));
        v_uav_0_rereasoning_command_button = new QPushButton(centralwidget);
        v_uav_0_rereasoning_command_button->setObjectName(QString::fromUtf8("v_uav_0_rereasoning_command_button"));
        v_uav_0_rereasoning_command_button->setGeometry(QRect(150, 390, 101, 22));
        v_uav_0_run_command_button = new QPushButton(centralwidget);
        v_uav_0_run_command_button->setObjectName(QString::fromUtf8("v_uav_0_run_command_button"));
        v_uav_0_run_command_button->setGeometry(QRect(60, 390, 80, 22));
        v_uav_local_network->setCentralWidget(centralwidget);
        menubar = new QMenuBar(v_uav_local_network);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 950, 28));
        v_uav_local_network->setMenuBar(menubar);
        statusbar = new QStatusBar(v_uav_local_network);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        v_uav_local_network->setStatusBar(statusbar);

        retranslateUi(v_uav_local_network);

        QMetaObject::connectSlotsByName(v_uav_local_network);
    } // setupUi

    void retranslateUi(QMainWindow *v_uav_local_network)
    {
        v_uav_local_network->setWindowTitle(QApplication::translate("v_uav_local_network", "MainWindow", nullptr));
        v_uav_0_camera_image->setText(QApplication::translate("v_uav_local_network", "<html><head/><body><p align=\"center\">camera</p></body></html>", nullptr));
        v_uav_0_init_button->setText(QApplication::translate("v_uav_local_network", "Init", nullptr));
        v_uav_0_label->setText(QString());
        v_uav_0_current_position_label->setText(QApplication::translate("v_uav_local_network", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Online Uav</span></p></body></html>", nullptr));
        v_uav_0_mission_id_edit->setText(QApplication::translate("v_uav_local_network", "0", nullptr));
        v_uav_0_mission_label->setText(QApplication::translate("v_uav_local_network", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Mission</span></p></body></html>", nullptr));
        v_uav_0_camera_open_button->setText(QApplication::translate("v_uav_local_network", "Open", nullptr));
        v_uav_0_mission_start_button->setText(QApplication::translate("v_uav_local_network", "Start", nullptr));
        v_uav_0_mission_id_label->setText(QApplication::translate("v_uav_local_network", "Mission ID:", nullptr));
        v_uav_0_camera_close_button->setText(QApplication::translate("v_uav_local_network", "Close", nullptr));
        v_uav_0_send_command_button->setText(QApplication::translate("v_uav_local_network", "Send", nullptr));
        v_uav_0_mission_id_label_2->setText(QApplication::translate("v_uav_local_network", "V_UAV_0:", nullptr));
        v_uav_0_mission_id_label_3->setText(QApplication::translate("v_uav_local_network", "V_UAV_1:", nullptr));
        v_uav_0_mission_id_edit_2->setText(QApplication::translate("v_uav_local_network", "0", nullptr));
        v_uav_0_mission_id_label_4->setText(QApplication::translate("v_uav_local_network", "V_UAV_2:", nullptr));
        v_uav_0_mission_id_edit_3->setText(QApplication::translate("v_uav_local_network", "0", nullptr));
        v_uav_0_mission_id_edit_4->setText(QApplication::translate("v_uav_local_network", "0", nullptr));
        v_uav_0_stop_command_button->setText(QApplication::translate("v_uav_local_network", "Stop", nullptr));
        clear_reasoning_button->setText(QApplication::translate("v_uav_local_network", "Clear", nullptr));
        clear_feedback_button->setText(QApplication::translate("v_uav_local_network", "Clear", nullptr));
        v_uav_0_rereasoning_command_button->setText(QApplication::translate("v_uav_local_network", "Rereasoning", nullptr));
        v_uav_0_run_command_button->setText(QApplication::translate("v_uav_local_network", "Run", nullptr));
    } // retranslateUi

};

namespace Ui {
    class v_uav_local_network: public Ui_v_uav_local_network {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_V_UAV_LOCAL_NETWORK_H

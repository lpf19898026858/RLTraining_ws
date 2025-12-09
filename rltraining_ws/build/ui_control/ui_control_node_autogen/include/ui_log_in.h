/********************************************************************************
** Form generated from reading UI file 'log_in.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOG_IN_H
#define UI_LOG_IN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_log_in
{
public:
    QPushButton *log_in_pushButton;
    QLabel *info_label;

    void setupUi(QWidget *log_in)
    {
        if (log_in->objectName().isEmpty())
            log_in->setObjectName(QString::fromUtf8("log_in"));
        log_in->resize(934, 590);
        log_in_pushButton = new QPushButton(log_in);
        log_in_pushButton->setObjectName(QString::fromUtf8("log_in_pushButton"));
        log_in_pushButton->setGeometry(QRect(290, 270, 231, 61));
        QFont font;
        font.setPointSize(20);
        log_in_pushButton->setFont(font);
        info_label = new QLabel(log_in);
        info_label->setObjectName(QString::fromUtf8("info_label"));
        info_label->setGeometry(QRect(230, 120, 481, 151));
        QFont font1;
        font1.setPointSize(11);
        info_label->setFont(font1);

        retranslateUi(log_in);

        QMetaObject::connectSlotsByName(log_in);
    } // setupUi

    void retranslateUi(QWidget *log_in)
    {
        log_in->setWindowTitle(QApplication::translate("log_in", "Form", nullptr));
        log_in_pushButton->setText(QApplication::translate("log_in", "log in", nullptr));
        info_label->setText(QApplication::translate("log_in", "Please Input the login buttom and work..", nullptr));
    } // retranslateUi

};

namespace Ui {
    class log_in: public Ui_log_in {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOG_IN_H

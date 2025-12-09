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
#include "log_in.h"
#include "ui_log_in.h"
#include <QMainWindow>
log_in::log_in(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::log_in)
{
    ui->setupUi(this);
}

log_in::~log_in()
{
    delete ui;
}


void log_in::on_log_in_pushButton_clicked()
{
   v_uav_local_network* l_v_uav = new v_uav_local_network;
   l_v_uav->set_network_name("local_server");
   l_v_uav->init_ros();
   l_v_uav->show();
 }


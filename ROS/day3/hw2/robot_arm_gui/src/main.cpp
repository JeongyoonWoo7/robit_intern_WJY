#include "rclcpp/rclcpp.hpp"
#include "robot_arm_gui/mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    MainWindow w;
    w.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}


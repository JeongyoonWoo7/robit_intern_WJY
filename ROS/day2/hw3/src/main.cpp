#include <QApplication>
#include "communicator/mainwindow.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWindow w;
    w.show();

  
    std::thread ros_spin([&w]() {
        rclcpp::spin(w.getQNode());
    });
    ros_spin.detach();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}


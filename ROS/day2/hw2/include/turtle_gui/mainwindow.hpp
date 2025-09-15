#pragma once
#include <QMainWindow>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void sendCmdVel(double linear, double angular);
    void onForward();
    void onBackward();
    void onLeft();
    void onRight();
    void drawTriangle();
    void drawSquare();
    void drawCircle();

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    QTimer *timer_;
};


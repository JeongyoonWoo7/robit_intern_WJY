#include "turtle_gui/mainwindow.hpp"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

   
    node_ = std::make_shared<rclcpp::Node>("qt_turtle_controller");
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    
    connect(ui->btnForward, &QPushButton::clicked, this, &MainWindow::onForward);
    connect(ui->btnBackward, &QPushButton::clicked, this, &MainWindow::onBackward);
    connect(ui->btnLeft, &QPushButton::clicked, this, &MainWindow::onLeft);
    connect(ui->btnRight, &QPushButton::clicked, this, &MainWindow::onRight);

    connect(ui->btnTriangle, &QPushButton::clicked, this, &MainWindow::drawTriangle);
    connect(ui->btnSquare,   &QPushButton::clicked, this, &MainWindow::drawSquare);
    connect(ui->btnCircle,   &QPushButton::clicked, this, &MainWindow::drawCircle);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::sendCmdVel(double linear, double angular)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    pub_->publish(msg);

    ui->labelCmdVel->setText(QString("linear: %1 , angular: %2").arg(linear).arg(angular));
}

void MainWindow::onForward() { sendCmdVel(2.0, 0.0); }
void MainWindow::onBackward(){ sendCmdVel(-2.0, 0.0); }
void MainWindow::onLeft()    { sendCmdVel(0.0, 2.0); }
void MainWindow::onRight()   { sendCmdVel(0.0, -2.0); }

void MainWindow::drawTriangle()
{
    for (int i=0; i<3; i++) {
        sendCmdVel(2.0, 0.0);
        rclcpp::sleep_for(std::chrono::seconds(1));
        sendCmdVel(0.0, 2.1); 
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

void MainWindow::drawSquare()
{
    for (int i=0; i<4; i++) {
        sendCmdVel(2.0, 0.0);
        rclcpp::sleep_for(std::chrono::seconds(1));
        sendCmdVel(0.0, 1.57);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

void MainWindow::drawCircle()
{
    sendCmdVel(2.0, 2.0);
    rclcpp::sleep_for(std::chrono::seconds(3));
    sendCmdVel(0.0, 0.0);
}


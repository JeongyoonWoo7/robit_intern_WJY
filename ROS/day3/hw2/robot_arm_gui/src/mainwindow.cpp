#include "robot_arm_gui/mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QPen>
#include <cmath>
#include <vector>



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    
    node_ = rclcpp::Node::make_shared("robot_arm_gui");
    pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/arm_joints", 10);

    
    connect(ui->sendButton, &QPushButton::clicked,
            this, &MainWindow::sendJointValues);
            
            
    
    node_->declare_parameter("robot_arm.link_lengths", std::vector<int>{120, 100, 80});
    node_->declare_parameter("robot_arm.angles", std::vector<int>{0,0,0});

    auto lengths = node_->get_parameter("robot_arm.link_lengths").as_integer_array();
    auto angles = node_->get_parameter("robot_arm.angles").as_integer_array();

    link_lengths = {(int)lengths[0], (int)lengths[1], (int)lengths[2]};

    ui->joint1->setValue((int)angles[0]);
    ui->joint2->setValue((int)angles[1]);
    ui->joint3->setValue((int)angles[2]);
    
    connect(ui->joint1, QOverload<int>::of(&QSpinBox::valueChanged), this, QOverload<>::of(&MainWindow::update));
    connect(ui->joint2, QOverload<int>::of(&QSpinBox::valueChanged), this, QOverload<>::of(&MainWindow::update));
    connect(ui->joint3, QOverload<int>::of(&QSpinBox::valueChanged), this, QOverload<>::of(&MainWindow::update));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::sendJointValues()
{
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {
        static_cast<float>(ui->joint1->value()),
        static_cast<float>(ui->joint2->value()),
        static_cast<float>(ui->joint3->value())
    };
    pub_->publish(msg);
}


void MainWindow::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    
    int baseX = width() / 2;
    int baseY = height() / 2 + 100;

    
    int l1 = link_lengths[0], l2 = link_lengths[1], l3 = link_lengths[2];

    
    double th1 = qDegreesToRadians((double)ui->joint1->value());
    double th2 = qDegreesToRadians((double)ui->joint2->value());
    double th3 = qDegreesToRadians((double)ui->joint3->value());

    
    QPoint p0(baseX, baseY);
    QPoint p1(p0.x() + l1 * cos(th1), p0.y() - l1 * sin(th1));
    QPoint p2(p1.x() + l2 * cos(th1 + th2), p1.y() - l2 * sin(th1 + th2));
    QPoint p3(p2.x() + l3 * cos(th1 + th2 + th3), p2.y() - l3 * sin(th1 + th2 + th3));

   
    painter.setPen(QPen(Qt::black, 4));
    painter.drawLine(p0, p1);
    painter.drawLine(p1, p2);
    painter.drawLine(p2, p3);

    painter.setBrush(Qt::red);
    painter.drawEllipse(p3, 6, 6);
}


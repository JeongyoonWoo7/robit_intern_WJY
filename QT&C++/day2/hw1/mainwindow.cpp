#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <QFile>
#include <QTextStream>
#include <QtMath>
#include <cmath>




MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      joint1(0), joint2(0), joint3(0),
      rotateDir1(0), rotateDir2(0), rotateDir3(0)
{
    ui->setupUi(this);

    setWindowTitle("3DOF Robot Arm");
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateAuto);

    ui->sliderJoint1->setRange(0, 360);
    ui->sliderJoint2->setRange(0, 360);
    ui->sliderJoint3->setRange(0, 360);

    ui->sliderJoint1->setValue(joint1);
    ui->sliderJoint2->setValue(joint2);
    ui->sliderJoint3->setValue(joint3);
}


MainWindow::~MainWindow() { delete ui; }

void MainWindow::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    QPoint base(width()/2, height()/2);

    int len1 = 120, len2 = 90, len3 = 60;

    // Joint1 끝점
    QPoint joint1Pos(
        base.x() + len1 * cos(qDegreesToRadians((double)joint1)),
        base.y() + len1 * sin(qDegreesToRadians((double)joint1))
    );

    // Joint2 끝점 (Joint1 + Joint2)
    QPoint joint2Pos(
        joint1Pos.x() + len2 * cos(qDegreesToRadians((double)(joint1 + joint2))),
        joint1Pos.y() + len2 * sin(qDegreesToRadians((double)(joint1 + joint2)))
    );

    // Joint3 끝점 (Joint1 + Joint2 + Joint3)
    QPoint joint3Pos(
        joint2Pos.x() + len3 * cos(qDegreesToRadians((double)(joint1 + joint2 + joint3))),
        joint2Pos.y() + len3 * sin(qDegreesToRadians((double)(joint1 + joint2 + joint3)))
    );

    // 링크 그리기
    p.setPen(QPen(Qt::black, 5));
    p.drawLine(base, joint1Pos);
    p.drawLine(joint1Pos, joint2Pos);
    p.drawLine(joint2Pos, joint3Pos);

    // 관절 강조
    p.setBrush(Qt::red);     p.drawEllipse(base, 5, 5);       // Base
    p.setBrush(Qt::blue);    p.drawEllipse(joint1Pos, 5, 5);  // Joint1
    p.setBrush(Qt::green);   p.drawEllipse(joint2Pos, 5, 5);  // Joint2
    p.setBrush(Qt::magenta); p.drawEllipse(joint3Pos, 5, 5);  // Joint3
}




void MainWindow::on_sliderJoint1_valueChanged(int value) {
    joint1 = value;
    update();
}

void MainWindow::on_sliderJoint2_valueChanged(int value) {
    joint2 = value;
    update();
}

void MainWindow::on_sliderJoint3_valueChanged(int value) {
    joint3 = value;
    update();
}

// Joint1
void MainWindow::on_btnCW1_clicked()  { rotateDir1 = +1; if (!timer->isActive()) timer->start(50); }
void MainWindow::on_btnCCW1_clicked() { rotateDir1 = -1; if (!timer->isActive()) timer->start(50); }

// Joint2
void MainWindow::on_btnCW2_clicked()  { rotateDir2 = +1; if (!timer->isActive()) timer->start(50); }
void MainWindow::on_btnCCW2_clicked() { rotateDir2 = -1; if (!timer->isActive()) timer->start(50); }

// Joint3
void MainWindow::on_btnCW3_clicked()  { rotateDir3 = +1; if (!timer->isActive()) timer->start(50); }
void MainWindow::on_btnCCW3_clicked() { rotateDir3 = -1; if (!timer->isActive()) timer->start(50); }

// Stop
void MainWindow::on_btnStop_clicked() {
    rotateDir1 = rotateDir2 = rotateDir3 = 0;
    timer->stop();
}



void MainWindow::on_btnSave_clicked() {
    QFile file("arm_state.txt");
    if (file.open(QIODevice::WriteOnly)) {
        QTextStream out(&file);
        out << joint1 << " " << joint2 << " " << joint3 << "\n";   
    }
}

void MainWindow::on_btnLoad_clicked() {
    QFile file("arm_state.txt");
    if (file.open(QIODevice::ReadOnly)) {
        QTextStream in(&file);
        in >> joint1 >> joint2 >> joint3;                         
        ui->sliderJoint1->setValue(joint1);
        ui->sliderJoint2->setValue(joint2);
        ui->sliderJoint3->setValue(joint3);                        
    }
    update();
}

void MainWindow::updateAuto() {
    if (rotateDir1 != 0) {
        joint1 = (joint1 + rotateDir1 + 361) % 361;
        ui->sliderJoint1->setValue(joint1);
    }

    if (rotateDir2 != 0) {
        joint2 = (joint2 + rotateDir2 + 361) % 361;
        ui->sliderJoint2->setValue(joint2);
    }

    if (rotateDir3 != 0) {
        joint3 = (joint3 + rotateDir3 + 361) % 361;
        ui->sliderJoint3->setValue(joint3);
    }

    update(); // 다시 그림
}




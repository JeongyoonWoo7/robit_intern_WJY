#include "communicator/mainwindow.hpp"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qnode = std::make_shared<QNode>();

    connect(ui->sendButton, &QPushButton::clicked,
            this, &MainWindow::onSendButtonClicked);

    connect(qnode.get(), &QNode::messageReceived,
            this, &MainWindow::updateLabel);
}

MainWindow::~MainWindow()
{
    delete ui;
    
}

void MainWindow::onSendButtonClicked()
{
    QString text = ui->lineEdit->text();
    qnode->publishMessage(text.toStdString());
    ui->lineEdit->clear();
}

void MainWindow::updateLabel(QString msg)
{
    ui->label->setText(msg);
}


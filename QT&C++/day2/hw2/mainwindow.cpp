#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QListWidget>
#include <QLCDNumber>
#include <QProgressBar>
#include <QSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("번호표 발급 시스템");

    // 초기 상태
    ui->lcdIssued->setDigitCount(4);
    ui->lcdIssued->display(0);
    ui->lcdNowServing->setDigitCount(4);
    ui->lcdNowServing->display(0);

    ui->progress->setRange(0, 100);
    ui->progress->setValue(0);

    ui->duration->setRange(1, 600);
    ui->duration->setValue(10);

    ui->waitLabel->setText("대기 인원: 0명");
    ui->serveLabel->setText("상담중: ");

    ui->callBtn->setEnabled(false);

    // 타이머
    timer_ = new QTimer(this);
    timer_->setInterval(100);
    connect(timer_, &QTimer::timeout, this, &MainWindow::onTick);

    // 시그널/슬롯
    connect(ui->issueBtn, &QPushButton::clicked, this, &MainWindow::issueTicket);
    connect(ui->callBtn,  &QPushButton::clicked, this, &MainWindow::startNext);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::issueTicket()
{
    Customer c;
    c.ticket  = (lastIssued_ % 9999) + 1;  // 1~9999 순환
    lastIssued_ = c.ticket;
    c.name    = ui->nameEdit->text().trimmed();
    if (c.name.isEmpty()) c.name = "손님";
    c.arrived = QDateTime::currentDateTime();

    // enqueue
    queue_.push_back(c);
    ui->lcdIssued->display(c.ticket);
    ui->nameEdit->clear();

    refreshList();
    updateWaitLabel();
    updateButtons();

    // 메모리 누적 방지: head가 많이 진행됐으면 앞부분 압축
    if (head_ > 256 && head_ * 2 > static_cast<int>(queue_.size()))
        compact();
}

void MainWindow::startNext()
{
    if (serving_.has_value() || head_ >= static_cast<int>(queue_.size()))
        return; 

    // front pop (O(1) : head_ 전진)
    serving_ = queue_[head_];
    ++head_;

    totalMs_   = ui->duration->value() * 1000;
    elapsedMs_ = 0;
    ui->progress->setValue(0);

    ui->lcdNowServing->display(serving_->ticket);
    ui->serveLabel->setText(
        QString("상담중: %1").arg(serving_->name)
    );


    timer_->start();

    refreshList();
    updateWaitLabel();
    updateButtons();
}

void MainWindow::onTick()
{
    elapsedMs_ += timer_->interval();
    const int pct = qBound(0, int(100.0 * elapsedMs_ / totalMs_), 100);
    ui->progress->setValue(pct);

    if (elapsedMs_ >= totalMs_) {
        timer_->stop();

        
        serving_.reset();
        ui->serveLabel->setText("현재 상담: ");

        updateButtons();

    }
}

void MainWindow::refreshList()
{
    ui->list->clear();
    for (int i = head_; i < static_cast<int>(queue_.size()); ++i) {
        const auto &c = queue_[i];
        const auto t  = c.arrived.time().toString("HH:mm:ss");
        ui->list->addItem(
            QString("%1 - %2 (%3)")
                .arg(c.ticket, 4, 10, QChar('0'))
                .arg(c.name)
                .arg(t)
        );
    }
}

void MainWindow::updateWaitLabel()
{
    int waiting = static_cast<int>(queue_.size()) - head_;
    if (waiting < 0) waiting = 0;
    ui->waitLabel->setText(QString("대기 인원: %1명").arg(waiting));
}

void MainWindow::updateButtons()
{
    const bool busy = serving_.has_value() && timer_->isActive();
    ui->issueBtn->setEnabled(true);

    int waiting = static_cast<int>(queue_.size()) - head_;
    ui->callBtn->setEnabled(!busy && waiting > 0);
}

void MainWindow::compact()
{
    // head_ 이전 요소들을 제거하여 메모리 회수
    queue_.erase(queue_.begin(), queue_.begin() + head_);
    head_ = 0;
}

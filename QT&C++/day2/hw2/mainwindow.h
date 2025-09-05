#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDateTime>
#include <vector>
#include <optional>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct Customer {
    int       ticket = 0;
    QString   name;
    QDateTime arrived;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void issueTicket();   // 번호표 발권
    void startNext();     // 다음 고객 호출(상담 시작)
    void onTick();        // 진행률 타이머 tick

private:
    // 내부 유틸
    void refreshList();       // 대기열 리스트뷰 갱신
    void updateWaitLabel();   // 대기 인원 표시 갱신
    void updateButtons();     // 버튼 활성/비활성 제어
    void compact();           // vector 앞부분 압축(메모리 회수)

    // 자료구조 (Queue = vector + head)
    std::vector<Customer> queue_;
    int head_ = 0;              // front 인덱스
    int lastIssued_ = 0;        // 마지막 발권 번호 (1~9999 순환)
    std::optional<Customer> serving_; // 현재 상담 중 고객

    // 타이머/시간
    QTimer *timer_ = nullptr;
    int totalMs_ = 0;
    int elapsedMs_ = 0;

    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

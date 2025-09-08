#pragma once
#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QTextEdit>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onLetterGroupClicked();
    void on_btn_Space_clicked();
    void on_btn_Enter_clicked();
    void on_btn_Backspace_clicked();
    void onShiftToggled(bool checked);  // Shift 버튼 토글

    void finalizePending();

private:
    Ui::MainWindow *ui;
    QString buffer_;


    QPushButton* pendingBtn_ = nullptr;
    QString pendingSet_;
    int pendingIndex_ = -1;
    QTimer commitTimer_;
    int multiTapMs_ = 800;

    bool shiftOn_ = true;  // true=대문자, false=소문자

    void connectLetterGroups();
    QString setFor(QPushButton* btn) const;
    void refreshText();
};

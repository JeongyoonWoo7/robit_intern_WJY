#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void on_sliderJoint1_valueChanged(int value);
    void on_sliderJoint2_valueChanged(int value);
    void on_sliderJoint3_valueChanged(int value);

    void on_btnCW1_clicked();
    void on_btnCCW1_clicked();
    void on_btnCW2_clicked();
    void on_btnCCW2_clicked();
    void on_btnCW3_clicked();
    void on_btnCCW3_clicked();
    void on_btnStop_clicked();

    void on_btnSave_clicked();
    void on_btnLoad_clicked();
    void updateAuto();

private:
    Ui::MainWindow *ui;  
    int joint1, joint2, joint3;
    int rotateDir1, rotateDir2, rotateDir3;
    QTimer *timer;
};

#endif // MAINWINDOW_H

#pragma once
#include <QMainWindow>
#include "communicator/qnode.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    
    std::shared_ptr<QNode> getQNode() { return qnode; }

private slots:
    void onSendButtonClicked();
    void updateLabel(QString msg);

private:
    Ui::MainWindow *ui;

    
    std::shared_ptr<QNode> qnode;
};


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QTimer>
#include <QVector>
#include <QPair>
#include <queue>


class GridWidget : public QWidget {
    Q_OBJECT
public:
    explicit GridWidget(bool useAStar, QWidget* parent=nullptr);

    void resetSearch();
    void clearMap();
    void startStop();

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void keyPressEvent(QKeyEvent*) override;

private slots:
    void step(); // 타이머 tick

private:
    // 설정
    const bool useAStar_;
    int W_ = 40, H_ = 25, cell_ = 20;
    int stepsPerTick_ = 12;

    // 맵/상태
    QVector<QVector<int>> grid_;              // 0=빈칸, 1=벽
    QPair<int,int> start_{2, H_-3};          // (x=row, y=col)
    QPair<int,int> goal_{W_-3, 2};

    QVector<QVector<double>> g_;
    QVector<QVector<bool>> closed_;
    QVector<QVector<QPair<int,int>>> parent_;

    struct Node { int x,y; double f,g; };
    struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
    std::priority_queue<Node, std::vector<Node>, Cmp> open_;

    QTimer timer_;
    bool running_ = false;
    bool finished_ = false;
    bool hasPath_ = false;

    // 유틸
    bool in(int x,int y) const { return 0<=x && x<H_ && 0<=y && y<W_; }
    double heuristic(int x,int y,int gx,int gy) const;
    QVector<QPair<int,int>> reconstruct() const;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=nullptr);
    ~MainWindow();
private:
    GridWidget* astar_;
    GridWidget* dijkstra_;
};

#endif // MAINWINDOW_H

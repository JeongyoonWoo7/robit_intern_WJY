#include "mainwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>
#include <limits>
#include <queue>

// ===== GridWidget =====
GridWidget::GridWidget(bool useAStar, QWidget* parent)
    : QWidget(parent), useAStar_(useAStar)
{
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(W_*cell_, H_*cell_);

    // 맵 초기화
    grid_ = QVector<QVector<int>>(H_, QVector<int>(W_, 0));
    resetSearch();

    connect(&timer_, &QTimer::timeout, this, &GridWidget::step);
    timer_.setInterval(15); // 속도
}

void GridWidget::resetSearch() {
    g_ = QVector<QVector<double>>(H_, QVector<double>(W_, std::numeric_limits<double>::infinity()));
    closed_ = QVector<QVector<bool>>(H_, QVector<bool>(W_, false));
    parent_ = QVector<QVector<QPair<int,int>>>(H_, QVector<QPair<int,int>>(W_, {-1,-1}));
    while(!open_.empty()) open_.pop();

    auto [sx,sy] = start_;
    auto [gx,gy] = goal_;
    if (!in(sx,sy) || !in(gx,gy) || grid_[sx][sy]==1 || grid_[gx][gy]==1) {
        finished_ = true; hasPath_ = false; running_ = false; timer_.stop();
        update(); return;
    }

    g_[sx][sy] = 0.0;
    double h0 = heuristic(sx,sy,gx,gy);
    open_.push({sx,sy, h0, 0.0});
    finished_ = false; hasPath_ = false;
    update();
}

void GridWidget::clearMap() {
    for(int i=0;i<H_;++i) for(int j=0;j<W_;++j) grid_[i][j]=0;
    resetSearch();
}

void GridWidget::startStop() {
    running_ = !running_;
    if (running_) timer_.start();
    else timer_.stop();
}

double GridWidget::heuristic(int x,int y,int gx,int gy) const {
    if (!useAStar_) return 0.0; // Dijkstra
    return std::abs(x-gx) + std::abs(y-gy); // Manhattan
}

QVector<QPair<int,int>> GridWidget::reconstruct() const {
    QVector<QPair<int,int>> path;
    auto [sx,sy]=start_; auto [gx,gy]=goal_;
    int cx=gx, cy=gy;
    if (!in(cx,cy) || parent_[cx][cy]==QPair<int,int>{-1,-1}) return path;
    while(!(cx==sx && cy==sy)){
        path.push_back({cx,cy});
        auto pr = parent_[cx][cy];
        cx=pr.first; cy=pr.second;
    }
    path.push_back({sx,sy});
    std::reverse(path.begin(), path.end());
    return path;
}

void GridWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), Qt::white);

    // 제목
    {
        QFont f = p.font(); f.setBold(true); p.setFont(f);
        p.setPen(Qt::black);
        QString title = useAStar_ ? "A*" : "Dijkstra";
        p.drawText(8, 18, title);
    }

    // 방문 영역(파란 톤), 벽(검정)
    for(int i=0;i<H_;++i){
        for(int j=0;j<W_;++j){
            QRect r(j*cell_, i*cell_, cell_, cell_);
            if (grid_[i][j]==1) {
                p.fillRect(r, Qt::black);
            } else if (closed_[i][j]) {
                p.fillRect(r, QColor(90,130,255,110));
            }
        }
    }

    // 경로(노랑)
    if (hasPath_) {
        auto path = reconstruct();
        if (!path.isEmpty()) {
            p.setRenderHint(QPainter::Antialiasing, true);
            p.setPen(QPen(Qt::yellow, cell_*0.7, Qt::SolidLine, Qt::RoundCap));
            for (int k=1;k<path.size();++k){
                auto [ax,ay]=path[k-1]; auto [bx,by]=path[k];
                QPoint A(ay*cell_+cell_/2, ax*cell_+cell_/2);
                QPoint B(by*cell_+cell_/2, bx*cell_+cell_/2);
                p.drawLine(A,B);
            }
        }
    }

    // 그리드 라인
    p.setPen(QColor(220,220,220));
    for(int y=0;y<=H_;++y) p.drawLine(0,y*cell_, W_*cell_, y*cell_);
    for(int x=0;x<=W_;++x) p.drawLine(x*cell_,0, x*cell_, H_*cell_);

    // 시작/목표
    auto drawCell = [&](QPair<int,int> pt, QColor c){
        auto [x,y]=pt;
        p.fillRect(QRect(y*cell_, x*cell_, cell_, cell_), c);
    };
    drawCell(start_, Qt::green);
    drawCell(goal_ , Qt::red);
}

void GridWidget::mousePressEvent(QMouseEvent* e) {
    int y = e->pos().x()/cell_, x = e->pos().y()/cell_;
    if(!in(x,y)) return;

    Qt::KeyboardModifiers m = e->modifiers();;
    if (m & Qt::ShiftModifier) {
        goal_ = {x,y};
    } else if (m & Qt::ControlModifier) {
        start_ = {x,y};
    } else {
        if (QPair<int,int>(x,y)!=start_ && QPair<int,int>(x,y)!=goal_)
            grid_[x][y] = !grid_[x][y];
    }
    resetSearch();
}

void GridWidget::keyPressEvent(QKeyEvent* e) {
    if (e->key()==Qt::Key_Space) startStop();
    if (e->key()==Qt::Key_R) { resetSearch(); }
    if (e->key()==Qt::Key_C) { clearMap(); }
    QWidget::keyPressEvent(e);
}

void GridWidget::step() {
    if (finished_) { timer_.stop(); running_=false; return; }

    int expanded = 0;
    const int dx[4]={-1,1,0,0};
    const int dy[4]={0,0,-1,1};
    auto [gx,gy]=goal_;

    while(expanded < stepsPerTick_ && !open_.empty()){
        Node cur = open_.top(); open_.pop();
        int x=cur.x, y=cur.y;
        if(!in(x,y) || closed_[x][y]) continue;
        closed_[x][y]=true; ++expanded;

        if(x==gx && y==gy){
            finished_ = true; hasPath_ = true; update(); return;
        }

        for(int k=0;k<4;++k){
            int nx=x+dx[k], ny=y+dy[k];
            if(!in(nx,ny) || grid_[nx][ny]==1) continue;
            if(closed_[nx][ny]) continue;
            double ng = g_[x][y] + 1.0;
            if(ng < g_[nx][ny]){
                g_[nx][ny]=ng;
                parent_[nx][ny]={x,y};
                double h = heuristic(nx,ny,gx,gy);
                open_.push({nx,ny, ng + h, ng});
            }
        }
    }
    update(); // paintEvent 호출 → 애니메이션
}

// ===== MainWindow =====
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("A* vs Dijkstra (QWidget + paintEvent)");
    auto *central = new QWidget(this);
    auto *h = new QHBoxLayout(central);

    astar_ = new GridWidget(/*useAStar=*/true);
    dijkstra_ = new GridWidget(/*useAStar=*/false);
    h->setSpacing(10);
    h->addWidget(astar_);
    h->addWidget(dijkstra_);

    setCentralWidget(central);
    resize(astar_->width()+dijkstra_->width()+60, std::max(astar_->height(), dijkstra_->height())+60);
}

MainWindow::~MainWindow() {}

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTextCursor>

#include <QFile>
#include <QTextStream>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connectLetterGroups();

    // Shift 버튼을 토글로
    ui->btn_Shift->setCheckable(true);
    connect(ui->btn_Shift, &QPushButton::toggled, this, &MainWindow::onShiftToggled);


    commitTimer_.setSingleShot(true);
    connect(&commitTimer_, &QTimer::timeout, this, &MainWindow::finalizePending);

    buffer_.clear();
    refreshText();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::connectLetterGroups() {
    connect(ui->btn_ABC,  &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
    connect(ui->btn_DEF,  &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
    connect(ui->btn_GHI,  &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
    connect(ui->btn_JKL,  &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
    connect(ui->btn_MNO,  &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
    connect(ui->btn_PQRS, &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
    connect(ui->btn_TUV,  &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
    connect(ui->btn_WXYZ, &QPushButton::clicked, this, &MainWindow::onLetterGroupClicked);
}

QString MainWindow::setFor(QPushButton* btn) const {
    if (btn == ui->btn_ABC)   return "ABC";
    if (btn == ui->btn_DEF)   return "DEF";
    if (btn == ui->btn_GHI)   return "GHI";
    if (btn == ui->btn_JKL)   return "JKL";
    if (btn == ui->btn_MNO)   return "MNO";
    if (btn == ui->btn_PQRS)  return "PQRS";
    if (btn == ui->btn_TUV)   return "TUV";
    if (btn == ui->btn_WXYZ)  return "WXYZ";
    return btn->text();
}

void MainWindow::refreshText() {
    QString composed = buffer_;
    if (pendingIndex_ >= 0 && !pendingSet_.isEmpty()) {
        QChar ch = pendingSet_.at(pendingIndex_);
        if (!shiftOn_) ch = ch.toLower();
        composed.append(ch);
    }
    ui->textEdit->setPlainText(composed);

    auto cursor = ui->textEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit->setTextCursor(cursor);
}

void MainWindow::finalizePending() {
    if (pendingIndex_ >= 0 && !pendingSet_.isEmpty()) {
        QChar ch = pendingSet_.at(pendingIndex_);
        if (!shiftOn_) ch = ch.toLower();
        buffer_.append(ch);
    }
    pendingBtn_  = nullptr;
    pendingSet_.clear();
    pendingIndex_ = -1;
    refreshText();
}

void MainWindow::onLetterGroupClicked() {
    QPushButton* btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    QString set = setFor(btn);
    if (pendingBtn_ == btn && pendingIndex_ >= 0) {
        pendingIndex_ = (pendingIndex_ + 1) % set.size();
        pendingSet_ = set;
    } else {
        finalizePending();
        pendingBtn_  = btn;
        pendingSet_  = set;
        pendingIndex_ = 0;
    }

    commitTimer_.start(multiTapMs_);
    refreshText();
}

void MainWindow::on_btn_Space_clicked() {
    finalizePending();
    buffer_.append(" ");
    refreshText();
}

void MainWindow::on_btn_Enter_clicked() {
    finalizePending();

    // 현재 텍스트 전체
    QString allText = buffer_;
    // 마지막 줄만 추출
    QStringList lines = allText.split("\n");
    QString lastLine = lines.isEmpty() ? "" : lines.last();

    // txt 파일 열기 (추가 모드)
    QFile file("output.txt");
    if (file.open(QIODevice::Append | QIODevice::Text)) {
        QTextStream out(&file);
        out.setCodec("UTF-8");           // 한글 깨짐 방지
        out << lastLine << "\n";
        file.close();
    } else {
        qDebug() << "파일 열기 실패!";
    }


    buffer_.append("\n");
    refreshText();
}


void MainWindow::on_btn_Backspace_clicked() {
    if (pendingIndex_ >= 0) {
        pendingBtn_ = nullptr;
        pendingSet_.clear();
        pendingIndex_ = -1;
    } else if (!buffer_.isEmpty()) {
        buffer_.chop(1);
    }
    refreshText();
}

void MainWindow::onShiftToggled(bool checked) {
    shiftOn_ = checked;   // true=대문자, false=소문자
    statusBar()->showMessage(checked ? "대문자" : "소문자", 1000);
    refreshText();
}

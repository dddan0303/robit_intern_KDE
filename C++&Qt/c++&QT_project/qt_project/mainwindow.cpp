#include "../include/mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>
#include <QComboBox>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>
#include <QJsonDocument>
#include <QDateTime>
#include <QDebug>
#include <climits>
#include <cmath>

// Qt5/Qt6 마우스 위치 호환
static inline QPointF mousePosF(const QMouseEvent* e) {
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    return e->position();
#else
    return e->localPos();
#endif
}

// 채점 파라미터
static constexpr int EDGE_THR = 190;
static constexpr double TOL_FRAC = 0.010;
static constexpr double SEARCH_RADIUS_FRAC = 0.012;
static constexpr double MIN_HIT_RATIO = 0.80;
static constexpr double ACC_WEIGHT = 0.80;
static constexpr double LEN_WEIGHT = 0.10;
static constexpr double COV_WEIGHT = 0.10;

// 네트워크 상수
static const qint64 HEARTBEAT_INTERVAL = 1000;
static const qint64 CONNECTION_TIMEOUT = 5000;

//=============================================================================
// DrawingCanvas 구현
//=============================================================================

DrawingCanvas::DrawingCanvas(QWidget *parent) : QFrame(parent) {
    setMouseTracking(true);
    setAttribute(Qt::WA_AcceptTouchEvents, true);
    loadGuideFromResource();
    rebuildScaledAndMask();
}

void DrawingCanvas::loadGuideFromResource() {
    QImage img(":/img/sample_line.png");
    if (img.isNull()) {
        emit statusText("리소스 이미지(:/img/sample_line.png) 로드 실패");
        guideOriginal = QImage();
        return;
    }
    guideOriginal = img.convertToFormat(QImage::Format_ARGB32_Premultiplied);
    emit statusText("가이드 이미지 로드 완료");
}

void DrawingCanvas::rebuildScaledAndMask() {
    if (guideOriginal.isNull()) return;
    QRectF area = guideTargetRect();
    if (area.isEmpty()) return;

    guideScaled = guideOriginal.scaled(area.size().toSize(),
                                      Qt::KeepAspectRatio,
                                      Qt::SmoothTransformation);

    // 윤곽 마스크 생성
    QImage g = guideScaled.convertToFormat(QImage::Format_ARGB32);
    maskScaled = QImage(g.size(), QImage::Format_Grayscale8);

    for (int y = 0; y < g.height(); ++y) {
        const QRgb* s = reinterpret_cast<const QRgb*>(g.constScanLine(y));
        uchar* d = maskScaled.scanLine(y);
        for (int x = 0; x < g.width(); ++x) {
            int gray = qGray(s[x]);
            d[x] = (gray < EDGE_THR) ? 255 : 0;
        }
    }
}

QRectF DrawingCanvas::guideTargetRect() const {
    QSizeF sz = size();
    double margin = 50.0;
    return QRectF(margin, margin, sz.width() - 2*margin, sz.height() - 2*margin);
}

QPoint DrawingCanvas::imagePointFromWidgetPoint(const QPointF& p) const {
    if (guideScaled.isNull()) return QPoint(-1, -1);

    QRectF area = guideTargetRect();
    QSizeF imgSz = guideScaled.size();
    QPointF tl(area.center().x() - imgSz.width()/2.0,
               area.center().y() - imgSz.height()/2.0);

    double x = p.x() - tl.x();
    double y = p.y() - tl.y();

    if (x < 0 || y < 0 || x >= imgSz.width() || y >= imgSz.height())
        return QPoint(-1, -1);
    return QPoint(x, y);
}

void DrawingCanvas::startGame() {
    resetStrokes();
    running = true;
    timer.restart();
    emit statusText("시작! 마우스로 선을 그어주세요!");
    update();
}

void DrawingCanvas::stopGame() {
    running = false;
    double s1 = scoreFor(myStrokes);
    double s2 = scoreFor(peerStrokes);
    emit scoresUpdated(s1, s2, (s1+s2)/2.0);
    emit statusText("채점 완료. 다시 시작하려면 [시작]을 누르세요.");
    update();
}

void DrawingCanvas::resetStrokes() {
    myStrokes.clear();
    peerStrokes.clear();
    drawing = false;
    peerDrawing = false;
    update();
}

void DrawingCanvas::paintEvent(QPaintEvent*) {
    QPainter g(this);
    g.fillRect(rect(), QColor("#fffdf5"));
    g.setRenderHint(QPainter::Antialiasing, true);

    QRectF area = guideTargetRect();
    g.setPen(QPen(QColor("#cfcfcf"), 2, Qt::DashLine));
    g.drawRoundedRect(area, 10, 10);

    // 가이드 이미지 그리기
    if (!guideScaled.isNull()) {
        QSizeF imgSz = guideScaled.size();
        QPointF tl(area.center().x() - imgSz.width()/2.0,
                   area.center().y() - imgSz.height()/2.0);
        g.setOpacity(1.0);
        g.drawImage(QRectF(tl, imgSz), guideScaled);
    } else {
        g.drawText(area, Qt::AlignCenter, "RESOURCE LOAD FAILED:\n:/img/sample_line.png");
    }

    // 내 선 그리기 (파란색)
    if (myStrokes.size() > 1) {
        g.setPen(QPen(QColor("#0066ff"), 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        for (int i = 1; i < myStrokes.size(); ++i) {
            g.drawLine(myStrokes[i-1], myStrokes[i]);
        }
    }

    // 상대방 선 그리기 (빨간색)
    if (peerStrokes.size() > 1) {
        g.setPen(QPen(QColor("#ff0066"), 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        for (int i = 1; i < peerStrokes.size(); ++i) {
            g.drawLine(peerStrokes[i-1], peerStrokes[i]);
        }
    }
}

void DrawingCanvas::mousePressEvent(QMouseEvent *e) {
    if (!running) return;

    if (e->button() == Qt::LeftButton) {
        drawing = true;
        myStrokes.push_back(mousePosF(e));
        emit drawStart();
        emit drawPoint(mousePosF(e));
    }
    update();
}

void DrawingCanvas::mouseMoveEvent(QMouseEvent *e) {
    if (!running || !drawing) return;

    myStrokes.push_back(mousePosF(e));
    emit drawPoint(mousePosF(e));
    update();
}

void DrawingCanvas::mouseReleaseEvent(QMouseEvent *e) {
    Q_UNUSED(e);
    if (drawing) {
        drawing = false;
        emit drawEnd();
    }
}

void DrawingCanvas::resizeEvent(QResizeEvent *e) {
    QFrame::resizeEvent(e);
    rebuildScaledAndMask();
    update();
}

// 네트워크 연동 함수들
void DrawingCanvas::addPeerPoint(const QPointF& point) {
    peerStrokes.push_back(point);
    update();
}

void DrawingCanvas::startPeerStroke() {
    peerDrawing = true;
}

void DrawingCanvas::endPeerStroke() {
    peerDrawing = false;
}

void DrawingCanvas::peerStartGame() {
    if (!running) startGame();
}

void DrawingCanvas::peerStopGame() {
    if (running) stopGame();
}

void DrawingCanvas::peerReset() {
    resetStrokes();
}

bool DrawingCanvas::isEdgePixelNear(const QPoint& ip, int maxR, double* outDistPx) const {
    if (ip.x()<0 || ip.y()<0 || ip.x()>=maskScaled.width() || ip.y()>=maskScaled.height())
        return false;

    if (maskScaled.pixelColor(ip).value() > 0) {
        if (outDistPx) *outDistPx = 0.0;
        return true;
    }

    int best2 = INT_MAX;
    for (int dy = -maxR; dy <= maxR; ++dy) {
        int yy = ip.y() + dy;
        if (yy < 0 || yy >= maskScaled.height()) continue;

        for (int dx = -maxR; dx <= maxR; ++dx) {
            int xx = ip.x() + dx;
            if (xx < 0 || xx >= maskScaled.width()) continue;
            if (dx*dx + dy*dy > maxR*maxR) continue;

            if (maskScaled.pixelColor(xx, yy).value() > 0) {
                int d2 = dx*dx + dy*dy;
                if (d2 < best2) best2 = d2;
            }
        }
    }

    if (best2 == INT_MAX) return false;
    if (outDistPx) *outDistPx = std::sqrt((double)best2);
    return true;
}

double DrawingCanvas::scoreFor(const QVector<QPointF>& pts) const {
    if (pts.size() < 5 || guideScaled.isNull()) return 0.0;

    const double ref = qMin(maskScaled.width(), maskScaled.height());
    const int maxR = qMax(1, int(ref * SEARCH_RADIUS_FRAC));
    const double tolPx = ref * TOL_FRAC;

    // 마스크 전체 선 픽셀 수
    int totalEdge = 0;
    for (int y = 0; y < maskScaled.height(); ++y) {
        for (int x = 0; x < maskScaled.width(); ++x) {
            if (maskScaled.pixelColor(x, y).value() > 0) totalEdge++;
        }
    }

    if (totalEdge == 0) return 0.0;

    // 샘플링 및 채점
    const int step = qMax(1, pts.size() / 900);
    double sumPart = 0.0;
    int cnt = 0;
    int hits = 0;

    std::unordered_set<long long> visited;
    auto key = [](int x, int y) -> long long {
        return ((long long)y << 32) | (unsigned int)x;
    };

    for (int i = 0; i < pts.size(); i += step) {
        QPoint ip = imagePointFromWidgetPoint(pts[i]);
        if (ip.x() < 0) continue;

        double distPx = 0.0;
        bool near = isEdgePixelNear(ip, maxR, &distPx);

        if (near && distPx <= tolPx) {
            hits++;
            sumPart += (tolPx - distPx) / tolPx;

            // 커버리지 계산을 위한 방문 픽셀 기록
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int xx = ip.x() + dx, yy = ip.y() + dy;
                    if (xx >= 0 && xx < maskScaled.width() &&
                        yy >= 0 && yy < maskScaled.height() &&
                        maskScaled.pixelColor(xx, yy).value() > 0) {
                        visited.insert(key(xx, yy));
                    }
                }
            }
        }
        cnt++;
    }

    if (cnt == 0) return 0.0;

    double accuracy = (double)hits / cnt;
    double precision = cnt > 0 ? sumPart / cnt : 0.0;
    double coverage = totalEdge > 0 ? (double)visited.size() / totalEdge : 0.0;

    if (accuracy < MIN_HIT_RATIO) return 0.0;

    return ACC_WEIGHT * precision + LEN_WEIGHT * accuracy + COV_WEIGHT * coverage;
}

//=============================================================================
// MainWindow 구현
//=============================================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , canvas(nullptr)
    , socket(new QUdpSocket(this))
    , heartbeatTimer(new QTimer(this))
    , connectionTimer(new QTimer(this))
    , peerPort(0)
    , myPort(0)
    , role(PLAYER_1)
    , connected(false)
    , lastHeartbeat(0)
{
    setWindowTitle("UDP 멀티플레이어 따라그리기");
    resize(1200, 800);

    auto *central = new QWidget(this);
    setCentralWidget(central);
    auto *root = new QVBoxLayout(central);

    // 네트워크 설정 UI
    setupNetworkUI();
    root->addWidget(networkGroup);

    // 게임 컨트롤 UI
    auto *ctrl = new QHBoxLayout();
    btnStart = new QPushButton("시작", this);
    btnReset = new QPushButton("리셋", this);
    labelScore = new QLabel("P1: 0 P2: 0 협동: 0", this);
    labelStatus = new QLabel("네트워크에 연결해주세요.", this);

    btnStart->setEnabled(false);
    btnReset->setEnabled(false);

    ctrl->addWidget(btnStart);
    ctrl->addWidget(btnReset);
    ctrl->addStretch(1);
    ctrl->addWidget(labelScore);

    // 캔버스
    canvas = new DrawingCanvas(this);
    canvas->setFrameShape(QFrame::StyledPanel);

    root->addLayout(ctrl);
    root->addWidget(canvas, 1);
    root->addWidget(labelStatus);

    // 시그널 연결
    connect(btnConnect, &QPushButton::clicked, this, &MainWindow::onConnect);
    connect(btnDisconnect, &QPushButton::clicked, this, &MainWindow::onDisconnect);
    connect(btnStart, &QPushButton::clicked, this, &MainWindow::onStart);
    connect(btnReset, &QPushButton::clicked, this, &MainWindow::onReset);

    connect(canvas, &DrawingCanvas::scoresUpdated, this, &MainWindow::onScoresUpdated);
    connect(canvas, &DrawingCanvas::statusText, this, &MainWindow::onStatus);

    // 네트워크 시그널
    connect(socket, &QUdpSocket::readyRead, this, &MainWindow::readPendingDatagrams);
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QUdpSocket::error),
            this, &MainWindow::onNetworkError);

    connect(heartbeatTimer, &QTimer::timeout, this, &MainWindow::sendHeartbeat);
    heartbeatTimer->setInterval(HEARTBEAT_INTERVAL);

    connect(connectionTimer, &QTimer::timeout, this, &MainWindow::checkConnection);
    connectionTimer->setInterval(CONNECTION_TIMEOUT / 2);

    // 캔버스와 네트워크 연결
    connect(canvas, &DrawingCanvas::drawPoint, this, &MainWindow::sendDrawPoint);
    connect(canvas, &DrawingCanvas::drawStart, this, &MainWindow::sendDrawStart);
    connect(canvas, &DrawingCanvas::drawEnd, this, &MainWindow::sendDrawEnd);
}

MainWindow::~MainWindow() {
    onDisconnect();
}

void MainWindow::setupNetworkUI() {
    networkGroup = new QGroupBox("네트워크 설정", this);
    auto *layout = new QVBoxLayout(networkGroup);

    // 내 정보
    auto *myLayout = new QHBoxLayout();
    myLayout->addWidget(new QLabel("내 IP:", this));
    editMyIP = new QLineEdit("127.0.0.1", this);
    myLayout->addWidget(editMyIP);
    myLayout->addWidget(new QLabel("포트:", this));
    spinMyPort = new QSpinBox(this);
    spinMyPort->setRange(1024, 65535);
    spinMyPort->setValue(12345);
    myLayout->addWidget(spinMyPort);

    // 상대방 정보
    auto *peerLayout = new QHBoxLayout();
    peerLayout->addWidget(new QLabel("상대 IP:", this));
    editPeerIP = new QLineEdit("127.0.0.1", this);
    peerLayout->addWidget(editPeerIP);
    peerLayout->addWidget(new QLabel("포트:", this));
    spinPeerPort = new QSpinBox(this);
    spinPeerPort->setRange(1024, 65535);
    spinPeerPort->setValue(12346);
    peerLayout->addWidget(spinPeerPort);

    // 역할 및 연결 버튼
    auto *ctrlLayout = new QHBoxLayout();
    ctrlLayout->addWidget(new QLabel("역할:", this));
    comboRole = new QComboBox(this);
    comboRole->addItems({"플레이어 1", "플레이어 2"});
    ctrlLayout->addWidget(comboRole);
    ctrlLayout->addStretch();

    btnConnect = new QPushButton("연결", this);
    btnDisconnect = new QPushButton("연결 해제", this);
    btnDisconnect->setEnabled(false);
    ctrlLayout->addWidget(btnConnect);
    ctrlLayout->addWidget(btnDisconnect);

    layout->addLayout(myLayout);
    layout->addLayout(peerLayout);
    layout->addLayout(ctrlLayout);

    // 플레이어 역할에 따라 포트 자동 설정
    connect(comboRole, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (index == 0) { // Player 1
            spinMyPort->setValue(12345);
            spinPeerPort->setValue(12346);
        } else { // Player 2
            spinMyPort->setValue(12346);
            spinPeerPort->setValue(12345);
        }
    });
}

void MainWindow::onConnect() {
    QString myIP = editMyIP->text();
    myPort = spinMyPort->value();
    peerIP = editPeerIP->text();
    peerPort = spinPeerPort->value();
    role = comboRole->currentIndex() == 0 ? PLAYER_1 : PLAYER_2;

    if (!socket->bind(QHostAddress(myIP), myPort)) {
        QMessageBox::critical(this, "연결 오류",
            QString("포트 %1 바인딩 실패: %2").arg(myPort).arg(socket->errorString()));
        return;
    }

    onStatus(QString("플레이어 %1로 %2:%3에서 대기중...").arg(role == PLAYER_1 ? "1" : "2").arg(myIP).arg(myPort));

    heartbeatTimer->start();
    connectionTimer->start();
    sendHeartbeat();

    updateConnectionState(false); // 연결 중 상태
}

void MainWindow::onDisconnect() {
    heartbeatTimer->stop();
    connectionTimer->stop();

    if (socket->state() != QAbstractSocket::UnconnectedState) {
        QJsonObject msg;
        msg["type"] = "disconnect";
        sendMessage(msg);
        socket->close();
    }

    connected = false;
    updateConnectionState(false);
    onStatus("연결이 해제되었습니다.");
}

void MainWindow::sendMessage(const QJsonObject& msg) {
    QJsonDocument doc(msg);
    QByteArray data = doc.toJson(QJsonDocument::Compact);

    if (socket->writeDatagram(data, QHostAddress(peerIP), peerPort) == -1) {
        qDebug() << "UDP 전송 실패:" << socket->errorString();
    }
}

void MainWindow::readPendingDatagrams() {
    while (socket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        QJsonParseError error;
        QJsonDocument doc = QJsonDocument::fromJson(datagram, &error);

        if (error.error == QJsonParseError::NoError) {
            processMessage(doc.object());
        }
    }
}

void MainWindow::processMessage(const QJsonObject& msg) {
    QString type = msg["type"].toString();

    if (type == "heartbeat") {
        if (!connected) {
            connected = true;
            updateConnectionState(true);
            onStatus(QString("플레이어 %1 연결됨!").arg(role == PLAYER_1 ? "2" : "1"));
        }
        lastHeartbeat = QDateTime::currentMSecsSinceEpoch();
    }
    else if (type == "draw_point") {
        QPointF point(msg["x"].toDouble(), msg["y"].toDouble());
        canvas->addPeerPoint(point);
    }
    else if (type == "draw_start") {
        canvas->startPeerStroke();
    }
    else if (type == "draw_end") {
        canvas->endPeerStroke();
    }
    else if (type == "game_start") {
        canvas->peerStartGame();
    }
    else if (type == "game_stop") {
        canvas->peerStopGame();
    }
    else if (type == "reset") {
        canvas->peerReset();
    }
    else if (type == "disconnect") {
        connected = false;
        updateConnectionState(false);
        onStatus("상대방이 연결을 끊었습니다.");
    }
}

void MainWindow::sendHeartbeat() {
    QJsonObject msg;
    msg["type"] = "heartbeat";
    msg["timestamp"] = QDateTime::currentMSecsSinceEpoch();
    sendMessage(msg);
}

void MainWindow::checkConnection() {
    if (connected && QDateTime::currentMSecsSinceEpoch() - lastHeartbeat > CONNECTION_TIMEOUT) {
        connected = false;
        updateConnectionState(false);
        onStatus("연결 타임아웃");
    }
}

void MainWindow::onNetworkError() {
    QMessageBox::critical(this, "네트워크 오류", socket->errorString());
    updateConnectionState(false);
}

void MainWindow::updateConnectionState(bool isConnected) {
    connected = isConnected;
    networkGroup->setEnabled(!isConnected);
    btnConnect->setEnabled(!isConnected);
    btnDisconnect->setEnabled(isConnected);
    btnStart->setEnabled(isConnected);
    btnReset->setEnabled(isConnected);
}

// 게임 메시지 전송 함수들
void MainWindow::sendDrawPoint(const QPointF& point) {
    QJsonObject msg;
    msg["type"] = "draw_point";
    msg["x"] = point.x();
    msg["y"] = point.y();
    sendMessage(msg);
}

void MainWindow::sendDrawStart() {
    QJsonObject msg;
    msg["type"] = "draw_start";
    sendMessage(msg);
}

void MainWindow::sendDrawEnd() {
    QJsonObject msg;
    msg["type"] = "draw_end";
    sendMessage(msg);
}

void MainWindow::sendGameStart() {
    QJsonObject msg;
    msg["type"] = "game_start";
    sendMessage(msg);
}

void MainWindow::sendGameStop() {
    QJsonObject msg;
    msg["type"] = "game_stop";
    sendMessage(msg);
}

void MainWindow::sendGameReset() {
    QJsonObject msg;
    msg["type"] = "reset";
    sendMessage(msg);
}

void MainWindow::onStart() {
    if (!canvas->isRunning()) {
        canvas->startGame();
        sendGameStart();
        btnStart->setText("종료/채점");
    } else {
        canvas->stopGame();
        sendGameStop();
        btnStart->setText("시작");
    }
}

void MainWindow::onReset() {
    canvas->resetStrokes();
    sendGameReset();
    labelScore->setText("P1: 0 P2: 0 협동: 0");
    onStatus("리셋됨. [시작]을 눌러 주세요.");
    btnStart->setText("시작");
}

void MainWindow::onScoresUpdated(double s1, double s2, double coop) {
    labelScore->setText(QString("P1: %1 P2: %2 협동: %3")
        .arg(s1,0,'f',1).arg(s2,0,'f',1).arg(coop,0,'f',1));
}

void MainWindow::onStatus(const QString& msg) {
    labelStatus->setText(msg);
}

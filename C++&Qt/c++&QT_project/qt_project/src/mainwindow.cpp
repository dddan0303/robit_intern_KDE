// 필요한 헤더 파일들을 포함
#include "mainwindow.h" // MainWindow 클래스 정의
#include "ui_mainwindow.h" // Qt Designer로 만든 UI 요소들
#include "drawingcanvas.h" // 그림 그리기 캔버스 클래스

#include <QJsonObject>// JSON 객체 처리
#include <QJsonDocument>// JSON 문서 변환
#include <QDateTime>// 날짜와 시간 처리
#include <QTimer> // 타이머 기능

namespace {
    // 게임 통과 조건: 진행률 70% 이상
    constexpr double kProgressThreshold = 0.70;
    // 게임 통과 조건: 협동 점수 80점 이상
    constexpr double kCoopThreshold = 80.0;
}

// MainWindow 클래스의 프로그램 시작할 때 실행됨
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), // 부모 클래스 생성자 호출
      ui(new Ui::MainWindow) // UI 객체 생성
{
    // UI 디자이너로 만든 화면 요소들을 실제로 배치
    ui->setupUi(this);

    // 그림 그리기 캔버스를 UI에서 가져와서 연결
    canvas = ui->drawingCanvas;

    // 시그널과 슬롯 연결 설정 (버튼 클릭 등의 이벤트 처리)
    setupConnections();

    // 게임 타이머 초기화
    setupTimer();
}

// MainWindow 클래스의 소멸자 - 프로그램 종료할 때 실행됨
MainWindow::~MainWindow()
{
    // UI 객체 메모리 해제
    delete ui;
}

// 시그널과 슬롯을 연결하는 함수 (이벤트와 처리 함수를 연결)
void MainWindow::setupConnections()
{
    // 시작 버튼을 누르면 onStart() 함수 실행
    connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::onStart);
    // 리셋버튼을 누르면 onReset() 함수 실행
    connect(ui->btnReset, &QPushButton::clicked, this, &MainWindow::onReset);
    // 연결 버튼을 누르면 onConnect() 함수 실행
    connect(ui->btnConnect, &QPushButton::clicked, this, &MainWindow::onConnect);

    // 점수가 업데이트되면 onScoresUpdated() 함수 실행
    connect(canvas, &DrawingCanvas::scoresUpdated, this, &MainWindow::onScoresUpdated);
    // 상태 메시지가 오면 onStatus() 함수 실행
    connect(canvas, &DrawingCanvas::statusText, this, &MainWindow::onStatus);

    // P1 라디오 버튼을 선택하면 (람다 함수 사용)
    connect(ui->rbP1, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            myRole = 1;                    // 내 역할을 플레이어1로 설정
            canvas->setLocalRole(1);       // 캔버스에도 알려줌
        }
    });

    // P2 라디오 버튼을 선택하면 (람다 함수 사용)
    connect(ui->rbP2, &QRadioButton::toggled, this, [this](bool on) {
        if (on) {
            myRole = 2;                    // 내 역할을 플레이어2로 설정
            canvas->setLocalRole(2);       // 캔버스에도 알려줌
        }
    });

    // 마우스를 누르면 상대방에게 전송 (람다 함수 사용)
    connect(canvas, &DrawingCanvas::localPress, this, [this](const QPointF& norm) {
        if (connected) {  // 네트워크 연결되어 있을 때만
            // JSON 형태로 데이터 구성: 타입="down", 역할=내역할, 좌표x,y
            sendJson(QJsonObject{{"t", "down"}, {"role", myRole}, {"x", norm.x()}, {"y", norm.y()}});
        }
    });

    // 마우스를 움직이면 상대방에게 전송
    connect(canvas, &DrawingCanvas::localMove, this, [this](const QPointF& norm) {
        if (connected) {  // 네트워크 연결되어 있을 때만
            // JSON 형태로 데이터 구성: 타입="move", 역할=내역할, 좌표x,y
            sendJson(QJsonObject{{"t", "move"}, {"role", myRole}, {"x", norm.x()}, {"y", norm.y()}});
        }
    });

    // 마우스를 놓으면 상대방에게 전송
    connect(canvas, &DrawingCanvas::localRelease, this, [this]() {
        if (connected) {  // 네트워크 연결되어 있을 때만
            // JSON 형태로 데이터 구성: 타입="up", 역할=내역할
            sendJson(QJsonObject{{"t", "up"}, {"role", myRole}});
        }
    });
}

// 게임 타이머를 초기화하는 함수
void MainWindow::setupTimer()
{
    // QTimer 객체 생성 (이 객체의 자식으로 설정)
    gameTimer = new QTimer(this);

    // 100밀리초(0.1초)마다 실행되도록 설정
    gameTimer->setInterval(100);

    // 타이머가 시간이 되면 onTick() 함수 실행
    connect(gameTimer, &QTimer::timeout, this, &MainWindow::onTick);
}

// "연결" 버튼을 눌렀을 때 실행되는 함수
void MainWindow::onConnect() {
    // 기존 UDP 소켓이 있으면 정리
    if (udp) {
        udp->close(); // 소켓 닫기
        udp->deleteLater(); // 메모리에서 제거 예약
        udp = nullptr; // 포인터 초기화
    }

    // 새로운 UDP 소켓 생성
    udp = new QUdpSocket(this);

    // UI에서 포트 번호들 가져오기
    localPort = static_cast<quint16>(ui->spinLocalPort->value()); // 내 포트 번호
    peerPort = static_cast<quint16>(ui->spinPeerPort->value()); // 상대방 포트 번호

    // UI에서 상대방 IP 주소 가져오기
    peerAddr = QHostAddress(ui->editPeerIp->text());

    // UDP 소켓을 내 포트에 바인딩 (네트워크 연결 준비)
    // ShareAddress  다른 프로그램과 포트 공유 허용
    // ReuseAddressHint 주소 재사용 허용
    if (!udp->bind(QHostAddress::AnyIPv4, localPort,
                   QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        // 바인딩 실패하면 에러 메시지 표시
        ui->labelStatus->setText(u8"포트 바인드 실패");
        return;
    }

    // 데이터가 도착하면 onReadyRead() 함수 실행하도록 연결
    connect(udp, &QUdpSocket::readyRead, this, &MainWindow::onReadyRead);

    connected = true;  // 연결 상태로 설정

    // 연결 성공 메시지 표시
    ui->labelStatus->setText(u8"연결됨. 역할=" + QString::number(myRole)
                         + u8", 상대=" + peerAddr.toString()
                         + ":" + QString::number(peerPort));

}

// JSON 데이터를 상대방에게 UDP로 전송하는 함수
void MainWindow::sendJson(const QJsonObject& obj) {
    if (!udp) return;  // UDP 소켓이 없으면 아무것도 하지 않음

    // JSON 객체를 압축된 텍스트로 변환
    const QByteArray dat = QJsonDocument(obj).toJson(QJsonDocument::Compact);

    // 상대방에게 UDP 패킷으로 전송
    udp->writeDatagram(dat, peerAddr, peerPort);
}

// 네트워크로부터 데이터가 도착했을 때 실행되는 함수
void MainWindow::onReadyRead() {
    // 받을 데이터가 있는 동안 계속 처리
    while (udp && udp->hasPendingDatagrams()) {
        // 받을 데이터 크기만큼 버퍼 준비
        QByteArray buf;
        buf.resize(int(udp->pendingDatagramSize()));

        // 송신자 정보를 저장할 변수들
        QHostAddress from;
        quint16 port = 0;

        // 실제로 데이터 읽기
        udp->readDatagram(buf.data(), buf.size(), &from, &port);

        // 받은 데이터를 JSON으로 변환
        const auto doc = QJsonDocument::fromJson(buf);
        if (!doc.isObject()) continue;  // JSON 객체가 아니면 건너뛰기

        const auto o = doc.object();// JSON 객체 가져오기
        const QString t = o.value("t").toString(); // 메시지 타입
        const int role = o.value("role").toInt(); // 플레이어 역할

        // 메시지 타입에 따라 다른 처리
        if (t == "down") {  // 상대방이 마우스를 눌렀을 때
            // 정규화된 좌표 정보 추출
            QPointF n(o.value("x").toDouble(), o.value("y").toDouble());
            // 캔버스에 상대방의 마우스 누르기 전달
            canvas->netPress(role, n);

        } else if (t == "move") {  // 상대방이 마우스를 움직였을 때
            // 정규화된 좌표 정보 추출
            QPointF n(o.value("x").toDouble(), o.value("y").toDouble());
            // 캔버스에 상대방의 마우스 움직임 전달
            canvas->netMove(role, n);

        } else if (t == "up") {  // 상대방이 마우스를 놓았을 때
            // 캔버스에 상대방의 마우스 놓기 전달
            canvas->netRelease(role);

        } else if (t == "start") {  // 상대방이 게임을 시작했을 때
            // 게임 시작 시간과 제한시간 정보 추출
            const qint64 epoch = static_cast<qint64>(o.value("epoch").toDouble(0));
            const int durMs = o.value("dur").toInt(canvas->suggestDurationMs());

            // 동기화된 카운트다운 시작
            startCountdownLocal(epoch, durMs);

            // 캔버스에 게임 시작 알림
            canvas->netStart();

            ui->btnStart->setText(u8"종료/채점");
            ui->labelStatus->setText(u8"상대가 시작함 (타임어택 동기화)");

        } else if (t == "timeout") { // 상대방쪽에서 시간 초과 발생
            stopCountdown(true); // 카운트다운 중지 (시간초과로)
            canvas->netStop(); // 캔버스에 게임 중지 알림
            ui->btnStart->setText(u8"시작");// 버튼 텍스트 원래대로

        } else if (t == "stop") {// 상대방이 게임을 수동 중지했을 때
            stopCountdown(false); // 카운트다운 중지 (수동으로)
            canvas->netStop(); // 캔버스에 게임 중지 알림
            ui->btnStart->setText(u8"시작"); // 버튼 텍스트 원래대로

        } else if (t == "reset_all") { // 상대방이 전체 리셋했을 때
            stopCountdown(false);// 카운트다운 중지
            canvas->setStage(1); // 스테이지를 1로 초기화
            canvas->netReset();// 캔버스 리셋

            // UI 초기화
            ui->labelScore->setText(u8"P1: 0  P2: 0  협동: 0");
            ui->btnStart->setText(u8"시작");
            ui->labelStatus->setText(u8"원격에서 리셋(스테이지 1)");

        } else if (t == "stage_set") { // 상대방이 스테이지를 변경했을 때
            const int st = o.value("stage").toInt(1); // 새 스테이지 번호
            canvas->setStage(st); // 스테이지 변경
            ui->labelScore->setText(u8"P1: 0  P2: 0  협동: 0"); // 점수 초기화

        } else if (t == "result") {  // 상대방으로부터 게임 결과를 받았을 때
            // 결과 데이터 추출
            const double prog = o.value("prog").toDouble(0.0); // 진행률
            const double s1 = o.value("s1").toDouble(0.0); // P1 점수
            const double s2 = o.value("s2").toDouble(0.0); // P2 점수
            const double coop = o.value("coop").toDouble(0.0); // 협동 점수
            const bool pass = o.value("pass").toBool(false); // 통과 여부
            const int curSt = o.value("stage").toInt(1); // 현재 스테이지
            const int nextSt = o.value("next").toInt(curSt);// 다음 스테이지
            const QString reason = o.value("reason").toString();// 실패 이유

            // 점수 UI 업데이트
            onScoresUpdated(s1, s2, coop);

            if (!pass) {  // 통과하지 못했을 때
                if (reason == "prog") {  // 진행률 부족
                    ui->labelStatus->setText(u8"미완료: 진행도 " + QString::number(prog*100.0,'f',1) + u8"%");
                } else if (reason == "coop") {  // 협동 점수 부족
                    ui->labelStatus->setText(u8"미완료: 협동 " + QString::number(coop,'f',1));
                } else {  // 기타 이유
                    ui->labelStatus->setText(u8"미완료");
                }
            } else {  // 통과했을 때
                if (curSt >= 3) {  // 스테이지 3을 통과하면 게임 완전 종료
                    ui->labelStatus->setText(u8"스테이지 3 통과! 게임 종료");
                } else {  // 다음 스테이지로 진행
                    canvas->setStage(nextSt);  // 다음 스테이지 설정
                    ui->labelStatus->setText(u8"스테이지 " + QString::number(curSt) + u8" 통과! → "
                                         + QString::number(nextSt) + u8" 준비됨. [시작]을 누르세요.");
                    ui->labelScore->setText(u8"P1: 0  P2: 0  협동: 0");  // 점수 초기화
                }
            }
        }
    }
}

// 시작 버튼을 눌렀을 때 실행되는 함수
void MainWindow::onStart() {
    if (!canvas->isRunning()) {  // 게임이 실행중이 아니면 시작
        // 현재 스테이지에 적합한 제한시간 계산
        durationMs = canvas->suggestDurationMs();

        // 현재 시간 (밀리초 단위)
        const qint64 now = QDateTime::currentMSecsSinceEpoch();

        // 로컬에서 카운트다운 시작
        startCountdownLocal(now, durationMs);

        // 캔버스에서 게임 시작
        canvas->startGame();

        // 버튼 텍스트 변경
        ui->btnStart->setText(u8"종료/채점");

        // 네트워크 연결되어 있으면 상대방에게도 시작 신호 전송
        if (connected) {
            QJsonObject o{{"t","start"}, {"epoch", double(now)}, {"dur", durationMs}, {"role", myRole}};
            sendJson(o);
        }
    } else {  // 게임이 실행중이면 수동으로 종료하고 채점
        finalizeAndBroadcastResult();
    }
}

// 리셋 버튼을 눌렀을 때 실행되는 함수
void MainWindow::onReset() {
    stopCountdown(false); // 카운트다운 중지
    canvas->setStage(1); // 스테이지를 1로 초기화
    canvas->resetStrokes();// 그린 모든 선들 지우기

    // UI 초기화
    ui->labelScore->setText(u8"P1: 0  P2: 0  협동: 0");
    ui->labelStatus->setText(u8"리셋됨. 스테이지 1 준비 완료. [시작]을 눌러 주세요.");
    ui->btnStart->setText(u8"시작");

    // 네트워크 연결되어 있으면 상대방에게도 리셋 신호 전송
    if (connected) sendJson(QJsonObject{{"t","reset_all"}, {"role", myRole}});
}

// 점수가 업데이트되었을 때 UI에 표시하는 함수
void MainWindow::onScoresUpdated(double s1, double s2, double coop) {
    // 점수를 소수점 1자리까지 표시하여 라벨 업데이트
    ui->labelScore->setText(QString(u8"P1: %1  P2: %2  협동: %3")
                            .arg(s1,0,'f',1) // s1을 소수점 1자리로
                            .arg(s2,0,'f',1) // s2를 소수점 1자리로
                            .arg(coop,0,'f',1)); // coop을 소수점 1자리로
}

// 상태 메시지를 UI에 표시하는 함수
void MainWindow::onStatus(const QString &msg) {
    ui->labelStatus->setText(msg);
}

// 로컬에서 카운트다운을 시작하는 함수
void MainWindow::startCountdownLocal(qint64 epochMs, int durMs) {
    startEpochMs = epochMs; // 시작 시간 저장
    durationMs = durMs; // 제한 시간 저장
    endEpochMs = startEpochMs + durationMs;// 종료 시간 계산
    timerRunning = true;  // 타이머 실행 상태로 설정

    // 현재 시간으로 타이머 라벨 업데이트
    updateTimerLabel(QDateTime::currentMSecsSinceEpoch());

    // 게임 타이머가 실행중이 아니면 시작
    if (!gameTimer->isActive()) gameTimer->start();
}

// 카운트다운을 중지하는 함수
void MainWindow::stopCountdown(bool byTimeout) {
    timerRunning = false;// 타이머 실행 상태 해제
    gameTimer->stop(); // 타이머 중지

    // 타이머 라벨을 00:00.0으로 초기화
    ui->labelTimer->setText(u8"남은 시간: 00:00.0");

    // 시간 초과로 중지되었고 네트워크 연결되어 있으면 상대방에게 알림
    if (byTimeout && connected) {
        sendJson(QJsonObject{{"t","timeout"}, {"role", myRole}});
    }
}

// 타이머가 주기적으로 호출하는 함수 (0.1초마다)
void MainWindow::onTick() {
    if (!timerRunning) return;  // 타이머가 실행중이 아니면 아무것도 하지 않음

    // 현재 시간
    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    // 시간이 다 되었으면
    if (now >= endEpochMs) {
        updateTimerLabel(endEpochMs); // 타이머를 정확히 0으로 맞춤
        finalizeAndBroadcastResult("timeout"); // 시간초과로 게임 종료
        return;
    }

    // 아직 시간이 남았으면 타이머 라벨 업데이트
    updateTimerLabel(now);
}

// 타이머 라벨을 현재 남은 시간으로 업데이트하는 함수
void MainWindow::updateTimerLabel(qint64 nowMs) {
    // 남은 시간 계산 (음수가 되지 않도록 최소 0으로 제한)
    qint64 remain = qMax<qint64>(0, endEpochMs - nowMs);

    // "남은 시간: MM:SS.D" 형태로 표시
    ui->labelTimer->setText(u8"남은 시간: " + fmtTimeMs(remain));
}

// 밀리초를 "분:초.소수점1자리" 형태의 문자열로 변환하는 함수
QString MainWindow::fmtTimeMs(qint64 ms) {
    if (ms < 0) ms = 0;  // 음수 방지

    qint64 totalSec = ms / 1000; // 전체 초 계산
    qint64 deci = (ms % 1000) / 100; // 소수점 첫째자리 (0.1초 단위)
    qint64 mm = totalSec / 60; // 분 계산
    qint64 ss = totalSec % 60;// 초 계산 (60으로 나눈 나머지)

    // "MM:SS.D" 형태로 반환
    return QString("%1:%2.%3")
        .arg(mm, 2, 10, QChar('0')) // 분을 2자리로, 빈자리는 0으로 채움
        .arg(ss, 2, 10, QChar('0')) // 초를 2자리로, 빈자리는 0으로 채움
        .arg(deci);  // 소수점 1자리
}

// 게임을 종료하고 결과를 처리하여 상대방에게 전송하는 함수
void MainWindow::finalizeAndBroadcastResult(const QString& reasonHint) {
    // 카운트다운 중지 (시간초과인지 수동인지에 따라)
    stopCountdown(reasonHint == "timeout");

    // 게임이 실행중이면 중지
    if (canvas->isRunning()) canvas->stopGame();

    // 버튼 텍스트를 "시작"으로 변경
    ui->btnStart->setText(u8"시작");

    // 현재 게임 결과 계산
    auto m = canvas->computeMetrics();

    // 통과 조건 확인: 진행률 70% 이상 AND 협동 점수 80점 이상
    bool pass = (m.progress >= kProgressThreshold) && (m.coop >= kCoopThreshold);

    QString reason;  // 실패 이유
    if (!pass) {
        // 진행률이 부족하면 "prog", 협동 점수가 부족하면 "coop"
        if (m.progress < kProgressThreshold) reason = "prog";
        else reason = "coop";
    }

    // 점수 UI 업데이트
    onScoresUpdated(m.s1, m.s2, m.coop);

    if (!pass) {  // 통과하지 못했을 때
        if (reason == "prog") {  // 진행률 부족
            ui->labelStatus->setText(u8"미완료: 진행도 " + QString::number(m.progress*100.0,'f',1) + u8"%");
        } else {  // 협동 점수 부족
            ui->labelStatus->setText(u8"미완료: 협동 " + QString::number(m.coop,'f',1));
        }
    } else {  // 통과했을 때
        if (canvas->stage() >= 3) {  // 스테이지 3을 통과하면 게임 완전 종료
            ui->labelStatus->setText(u8"스테이지 3 통과! 게임 종료");
        } else {  // 다음 스테이지로 진행
            int nextStage = canvas->stage() + 1;  // 다음 스테이지 번호
            canvas->setStage(nextStage);// 스테이지 변경

            // 통과 메시지 표시
            ui->labelStatus->setText(u8"스테이지 " + QString::number(nextStage-1) + u8" 통과! → "
                                 + QString::number(nextStage) + u8" 준비됨. [시작]을 누르세요.");
            ui->labelScore->setText(u8"P1: 0  P2: 0  협동: 0");  // 점수 초기화

            // 네트워크 연결되어 있으면 상대방에게 스테이지 변경 알림
            if (connected) {
                sendJson(QJsonObject{{"t","stage_set"}, {"stage", nextStage}});
            }
        }
    }

    // 네트워크 연결되어 있으면 상대방에게 결과 전송
    if (connected) {
        QJsonObject r{
            {"t","result"}, {"role", myRole},
            // 현재 스테이지 (통과했고 1보다 크면 -1, 아니면 현재 그대로)
            {"stage", canvas->stage() - (pass && canvas->stage() > 1 ? 1 : 0)},
            // 다음 스테이지 (통과하면 현재+1, 실패하면 현재 유지, 최대 3)
            {"next", pass ? std::min(3, canvas->stage()) : canvas->stage()},
            {"prog", m.progress}, {"s1", m.s1}, {"s2", m.s2}, {"coop", m.coop},
            {"pass", pass}, {"reason", reason}
        };
        sendJson(r);
    }
}

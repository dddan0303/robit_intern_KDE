#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow> // 메인 윈도우 기능을 사용하기 위해
#include <QUdpSocket>  // UDP 네트워크 통신 기능을 위해
#include <QHostAddress>// IP 주소를 저장하고 관리하기 위해
#include <QJsonObject> // JSON 형태의 데이터를 주고받기 위해

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }  // UI 디자이너로 만든 화면 요소들의 클래스 선언
class QTimer;                       // Qt 타이머 클래스 선언 (헤더 포함 대신 사용)
QT_END_NAMESPACE

// 그림 그리기 캔버스 클래스
class DrawingCanvas;

// MainWindow 클래스: 프로그램의 메인 화면을 관리하는 클래스
// QMainWindow를 상속받아서 기본적인 윈도우 기능들을 모두 사용할 수 있음
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // 생성자: MainWindow 객체를 만들 때 자동으로 실행되는 함수
    // parent는 이 윈도우의 부모 위젯을 지정
    explicit MainWindow(QWidget *parent = nullptr);

    //MainWindow 객체가 삭제될 때 자동으로 실행되는 함수
    // 메모리 정리와 리소스 해제 작업을 수행
    ~MainWindow();

private slots:
    //버튼을 누르거나 이벤트가 발생했을 때 자동으로 호출되는 함수들
    void onStart(); // "게임 시작" 버튼을 눌렀을 때 실행
    void onReset(); // 리셋버튼을 눌렀을 때 실행
    void onConnect(); // 연결 버튼을 눌렀을 때 실행
    void onReadyRead();  // 네트워크로 데이터가 도착했을 때 실행
    void onScoresUpdated(double s1, double s2, double coop); // 게임 점수가 바뀔 때 실행
    void onStatus(const QString &msg); // 상태 메시지를 화면에 표시할 때 실행
    void onTick(); // 타이머가 1초마다 실행할 때 호출

private:

    Ui::MainWindow *ui;    // 큐티 디자이너로 만든 버튼, 라벨 등의 UI 요소들에 접근하는 포인터
    DrawingCanvas *canvas;  // 실제로 그림을 그릴 수 있는 캔버스 영역의 포인터

    QUdpSocket *udp = nullptr; // UDP 방식으로 네트워크 통신을 하는 소켓 객체
    QHostAddress peerAddr; // 상대방 컴퓨터의 IP 주소를 저장하는 변수
    quint16 localPort = 45454; // 내 컴퓨터에서 사용할 포트 번호 (45454번)
    quint16 peerPort = 45455; // 상대방 컴퓨터의 포트 번호 (45455번)
    bool connected = false; // 현재 상대방과 연결되어 있는지 확인하는 불린 변수
    int myRole = 1; // 내가 플레이어1인지 2인지 구분하는 변수

    qint64 startEpochMs = 0; // 게임이 시작된 시간 (밀리초 단위로 저장)
    qint64 endEpochMs = 0;  // 게임이 끝나야 하는 시간 (밀리초 단위로 저장)
    int durationMs = 60000; // 게임 제한 시간 (60초 = 60000밀리초)
    bool timerRunning = false; // 현재 타이머가 돌아가고 있는지 확인하는 불린 변수
    QTimer *gameTimer = nullptr; // 1초마다 시간을 체크하고 화면을 업데이트하는 타이머 객체

    void setupConnections();// 시그널과 슬롯을 연결하는 초기 설정 함수
    void setupTimer();// 게임 타이머를 초기화하고 설정하는 함수
    void sendJson(const QJsonObject& obj); // JSON 형태의 데이터를 상대방에게 전송하는 함수
    void startCountdownLocal(qint64 epochMs, int durMs); // 내 컴퓨터에서 카운트다운을 시작하는 함수
    void stopCountdown(bool byTimeout); // 카운트다운을 중지하는 함수 (시간초과 또는 수동 중지)
    void updateTimerLabel(qint64 nowMs); // 화면의 타이머 표시를 현재 시간으로 업데이트하는 함수
    QString fmtTimeMs(qint64 ms);// 밀리초를 "분:초" 형태의 문자열로 변환하는 함수
    void finalizeAndBroadcastResult(const QString& reasonHint = ""); // 게임 결과를 정리하고 상대방에게 전송하는 함수
};

#endif

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Qt 기본 위젯들
#include <QMainWindow> // 메인 윈도우 클래스
#include <QUdpSocket> // UDP 소켓 통신
#include <QHostAddress>// 네트워크 주소 처리
#include <QMessageBox> // 메시지 박스 (알림)
#include <QTime>// 시간 처리 (메시지 시간체크용)
#include <QScrollBar> // 채팅창 스크롤 제어

// 대화상자 생성용 위젯들
#include <QDialog>// 연결 설정 대화상자
#include <QVBoxLayout> // 수직 레이아웃
#include <QHBoxLayout> // 수평 레이아웃용
#include <QGridLayout> // 격자 레이아웃
#include <QLineEdit> // 텍스트 입력 필드
#include <QSpinBox> // 숫자 입력 스핀박스
#include <QPushButton> // 버튼
#include <QLabel> // 라벨

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // 생성자: 메인 윈도우 초기화
    MainWindow(QWidget *parent = nullptr);
    // 소멸자: UDP 소켓 및 UI 리소스 정리
    ~MainWindow();

private slots:
    // 네트워크 연결 설정 대화상자 표시
    void showConnectionDialog();
    // UDP 소켓을 생성하고 네트워크에 연결
    void connectToNetwork();
    // 사용자가 입력한 메시지를 UDP로 전송
    void sendMessage();
    // UDP 소켓에서 수신된 데이터 처리
    void readyRead();

private:
    // UI 초기 설정 (버튼 연결, 초기 상태 등)
    void initializeUI();
    // 연결 설정 입력을 위한 대화상자 생성
    QDialog* createConnectionDialog();

    //ui 관련
    Ui::MainWindow *ui; // 생성된 UI 객체

    QUdpSocket *udpSocket; // UDP 소켓 수신용
    QString targetIP; // 상대방의 IP 주소
    quint16 myPort; // 내가 사용할 포트 번호
    quint16 targetPort; // 상대방의 포트 번호
    bool isConnected; // 현재 연결 상태

    QLineEdit *tempIpInput;// 상대방 IP 입력 필드
    QSpinBox *tempMyPortInput;  // 내 포트번호 입력
    QSpinBox *tempTargetPortInput; // 상대방 포트번호 입력
};

#endif // MAINWINDOW_H

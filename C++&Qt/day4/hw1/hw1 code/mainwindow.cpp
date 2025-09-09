#include "mainwindow.h"
#include "ui_mainwindow.h"

// 생성자: 메인 윈도우 초기화
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), udpSocket(nullptr), isConnected(false)
{
    // UI 설정 및 초기화
    ui->setupUi(this);
    initializeUI();
}

// 소멸자: 메모리 정리
MainWindow::~MainWindow()
{
    // UDP 소켓이 존재하면 연결 종료하고 메모리 해제
    if (udpSocket) {
        udpSocket->close();
        delete udpSocket;
    }
    // UI 메모리 해제
    delete ui;
}

// UI 초기 설정 함수
void MainWindow::initializeUI()
{
    ui->chatDisplay->append("UDP 채팅 프로그램");

    // 버튼 클릭 이벤트 연결
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::showConnectionDialog);
    connect(ui->sendButton, &QPushButton::clicked, this, &MainWindow::sendMessage);
    // Enter 키를 누르면 메시지 전송
    connect(ui->messageInput, &QLineEdit::returnPressed, this, &MainWindow::sendMessage);

    // 초기에는 메시지 전송 비활성화
    ui->sendButton->setEnabled(false);
    ui->messageInput->setEnabled(false);
}

// 연결 설정 대화상자 생성
QDialog* MainWindow::createConnectionDialog()
{
    // 새로운 대화상자 생성
    QDialog *dialog = new QDialog(this);
    // 대화상자 윗 부분 네이밍 설정
    dialog->setWindowTitle("네트워크 연결 설정");
    // 대화 상자 크기를 400,220 픽셀로 고정
    dialog->setFixedSize(400, 220);
    dialog->setModal(true); // 네트워크 설정 창이 열려 있으면 창을 사용불가 하게 모달창으로 설정

    // 메인 레이아웃 설정
    // 윗제들을 세로로 배치 하기 위해 수직 레이아웃 생성
    QVBoxLayout *mainLayout = new QVBoxLayout(dialog);
    // 위젯들 사이의 간격을 15픽셀로 설정
    mainLayout->setSpacing(15);
    // 위젯들 여백을 20 픽셀로 동일 시켜 정렬 시킴
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // 격자 레이아웃으로 라벨과 입력 필드 정렬
    // 표 형식으로 위젯 배치 하기 위해 격자 레이아웃 생성
    QGridLayout *gridLayout = new QGridLayout();
    // 격자 레이아웃에 간격을 10픽셀로 설정
    gridLayout->setSpacing(10);
    // 첫 번째 열을 최소 너비를 120 픽셀로 설정
    gridLayout->setColumnMinimumWidth(0, 120);
    // 두번째 열은 남은 공간 다 차지하게
    gridLayout->setColumnStretch(1, 1);

    // 상대방 IP 주소 입력
    // 연결할 상대의 IP를 입력할 텍스트 적을 수 있는 행
    QLabel *ipLabel = new QLabel("상대방 IP:");
    //0행 (첫번째)칸에 생성성
    gridLayout->addWidget(ipLabel, 0, 0);
    // 텍스트 입력 필드 생성
    tempIpInput = new QLineEdit("127.0.0.1"); // 예시로 이렇게 입력해라 예시 기본값
    //0행 1열에 생성
    gridLayout->addWidget(tempIpInput, 0, 1);

    // 내 포트 번호 입력
    // 나의 포트 번호를 입력할 텍스트 적을 수 있는 행
    QLabel *myPortLabel = new QLabel("내 포트:");
    gridLayout->addWidget(myPortLabel, 1, 0);
    // 숫자 입력을 할 수 있는 스핀 박스 생성
    tempMyPortInput = new QSpinBox();
    // 숫자 입력을 1024 부터 65535 까지 입력 가능하게 설정
    tempMyPortInput->setRange(1024, 65535);
    tempMyPortInput->setValue(12345); // 기본값을 12345로 설정
    gridLayout->addWidget(tempMyPortInput, 1, 1);

    // 상대방 포트 번호 입력
    QLabel *targetPortLabel = new QLabel("상대방 포트:");
    gridLayout->addWidget(targetPortLabel, 2, 0);

    tempTargetPortInput = new QSpinBox();
    tempTargetPortInput->setRange(1024, 65535);
    tempTargetPortInput->setValue(12346); // 기본값
    gridLayout->addWidget(tempTargetPortInput, 2, 1);

    mainLayout->addLayout(gridLayout);
    mainLayout->addSpacing(15);

    // 확인/취소 버튼 레이아웃
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    QPushButton *cancelButton = new QPushButton("취소");
    QPushButton *okButton = new QPushButton("확인");

    buttonLayout->addStretch();
    buttonLayout->addWidget(cancelButton);
    buttonLayout->addSpacing(10);
    buttonLayout->addWidget(okButton);
    mainLayout->addLayout(buttonLayout);

    // 버튼 클릭 이벤트 연결
    connect(cancelButton, &QPushButton::clicked, dialog, &QDialog::reject);
    connect(okButton, &QPushButton::clicked, dialog, &QDialog::accept);

    return dialog;
}

/// 연결 대화상자 표시
void MainWindow::showConnectionDialog()
{
    QDialog *dialog = createConnectionDialog(); //여기서 호출함 연결 설정 버튼 누르면 내부적으로 호출

    // 사용자가 확인 버튼을 누른 경우
    if (dialog->exec() == QDialog::Accepted) {
        // 입력된 값들을 멤버 변수에 저장
        targetIP = tempIpInput->text(); //상대 IP
        myPort = tempMyPortInput->value(); //자신 개방 포트
        targetPort = tempTargetPortInput->value(); // 상대방 개방 포트
        // 네트워크 연결 시도
        connectToNetwork();
    }

    // 대화상자 메모리 해제
    dialog->deleteLater();
}

// 네트워크에 연결
void MainWindow::connectToNetwork()
{
    // 기존 소켓이 있으면 정리, 중복되면 메모리 관리가 안 되기에
    if (udpSocket) {
        udpSocket->close();
        delete udpSocket;
    }

    // 새로운 UDP 소켓 생성
    udpSocket = new QUdpSocket(this);

    // 지정된 포트로 바인딩 시도
    if (udpSocket->bind(QHostAddress::Any, myPort)) {
        // 데이터 수신 이벤트 연결
        connect(udpSocket, &QUdpSocket::readyRead, this, &MainWindow::readyRead);

        // 연결 상태 업데이트
        isConnected = true;
        ui->statusLabel->setText(QString("연결됨 - 내 포트: %1, 상대방: %2:%3")
                                     .arg(myPort).arg(targetIP).arg(targetPort));

        // 메시지 전송 기능 활성화
        ui->sendButton->setEnabled(true);
        ui->messageInput->setEnabled(true);
        ui->messageInput->setFocus();

        // 연결되고 있는지 메시지 출력
        ui->chatDisplay->append(QString("포트 %1에서 연결 대기 중...").arg(myPort));
    } else {
        // 연결 실패 시 경고 메시지
        QMessageBox::warning(this, "연결 오류", QString("포트 %1에 바인딩 실패").arg(myPort));
    }
}

// 메시지 전송
void MainWindow::sendMessage()
{
    // 소켓이 없거나 입력된 메시지가 비어있으면 리턴
    if (!udpSocket || ui->messageInput->text().trimmed().isEmpty()) {
        return;
    }

    // 입력된 메시지와 현재 시간 가져오기
    QString message = ui->messageInput->text().trimmed();
    QString currentTime = QTime::currentTime().toString("hh:mm:ss");
    QByteArray data = message.toUtf8(); // UTF-8로 인코딩

    // UDP 데이터그램 전송
    qint64 result = udpSocket->writeDatagram(data, QHostAddress(targetIP), targetPort);

    if (result != -1) {
        // 전송 성공 시 채팅창에 표시
        ui->chatDisplay->append(QString("[%1] 나: %2").arg(currentTime, message));
        ui->messageInput->clear(); // 입력 필드 비우기

        // 채팅창을 가장 아래로 스크롤
        ui->chatDisplay->verticalScrollBar()->setValue(
            ui->chatDisplay->verticalScrollBar()->maximum());
    } else {
        // 전송 실패 시 경고 메시지
        QMessageBox::warning(this, "전송 오류", "메시지 전송 실패");
    }
}

// 메시지 수신 처리
void MainWindow::readyRead()
{
    // 받을 데이터가 있는 동안 반복
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray buffer;
        buffer.resize(udpSocket->pendingDatagramSize());

        QHostAddress sender;
        quint16 senderPort;

        // 데이터그램 읽기
        udpSocket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

        // 받은 메시지를 문자열로 변환
        QString message = QString::fromUtf8(buffer);
        QString currentTime = QTime::currentTime().toString("hh:mm:ss");

        // 채팅창에 받은 메시지 표시
        ui->chatDisplay->append(QString("[%1] 상대방: %2").arg(currentTime, message));

        // 채팅창을 가장 아래로 스크롤
        ui->chatDisplay->verticalScrollBar()->setValue(
            ui->chatDisplay->verticalScrollBar()->maximum());
    }
}

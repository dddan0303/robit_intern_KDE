#ifndef DRAWINGCANVAS_H
#define DRAWINGCANVAS_H

// 화면에 보이는 프레임 위젯을 만들기 위해
#include <QFrame>
// 동적 배열 컨테이너
#include <QVector>
// 좌표 x,y 저장
#include <QPointF>
// 스테이지에 따른 이미지들의 데이터를 처리 하기 위한 함수
#include <QImage>
// 타임어택을 위해서 시간을 측정하기 위한 함수
#include <QElapsedTimer>

//마우스로 하는 모든 동작 처리
class QMouseEvent;
// 창 크기 변경 했을 때 동작 처리
class QResizeEvent;
// 화면에 그리리기 처리
class QPaintEvent;

// 2인용 그림 그리기 게임의 캔버스 DrawingCanvas 클래스
class DrawingCanvas : public QFrame
{
    Q_OBJECT  // qt에서  시그널/슬롯 시스템을 사용하기 위한 매크로

public:
    // 생성자: parent는 이 위젯의 부모 위젯
    explicit DrawingCanvas(QWidget *parent = nullptr);

    // 스테이지 번호를 각각 설정해주고
    void setStage(int s);
    // 현재 스테이지 번호를 반환
    int stage() const { return curStage; }

   //게임을 시작 했을때, 정지 시켰을때, 그린 선들을 다 리셋 시킬때
    void startGame();
    void stopGame();
    void resetStrokes();
    bool isRunning() const { return running; }
    // 게임이 실행중인지 확인

    // 1=플레이어1, 2=플레이어2
    void setLocalRole(int r) { localRole = r; }


    struct Metrics {
        // 전체 진행률 (0~1 사이값)
        double progress = 0.0;
        // 플레이어1,2 점수 (0~100점)
        double s1 = 0.0;
        double s2 = 0.0;
        // 협동 점수 (0~100점)
        double coop = 0.0;
    };

    // 현재 게임 상태의 점수들을 계산해서 반환
    Metrics computeMetrics() const;

    // 현재 스테이지에 적합한 제한시간을 밀리초 단위로 변환
    int suggestDurationMs() const;

protected:
    // qt 이벤트
    void paintEvent(QPaintEvent *event) override;       // 화면을 다시 그릴 때
    void mousePressEvent(QMouseEvent *event) override;  // 마우스 버튼을 누를 때
    void mouseMoveEvent(QMouseEvent *event) override;   // 마우스를 움직일 때
    void mouseReleaseEvent(QMouseEvent *event) override;// 마우스 버튼을 놓을 때
    void resizeEvent(QResizeEvent *event) override;     // 창 크기가 바뀔 때

public slots:
    // udp 통신을 통해서 상대방과 연결되었을때 상대방의 qt 이벤트
    void netPress(int role, const QPointF& norm);   // 상대방이 마우스를 눌렀을 때  norm은 정규화된 좌표 (0~1 사이값)
    void netMove(int role, const QPointF& norm);    // 상대방이 마우스를 움직였을 때
    void netRelease(int role);                      // 상대방이 마우스를 놓았을 때
    void netStart();                                // 상대방이 게임을 시작했을 때
    void netStop();                                 // 상대방이 게임을 중지했을 때
    void netReset();                                // 상대방이 리셋했을 때

signals:
    // 내 입력을 상대방에게 전송하기 위해 발생시키는 시그널
    void localPress(const QPointF& norm);   // 내가 마우스를 누를 때
    void localMove(const QPointF& norm);    // 내가 마우스를 움직일 때
    void localRelease();                    // 내가 마우스를 놓을 때

    // 시간초과, 점수, 등 ui 애서 화면이 바뀌어야 할 떄 보낼 시그널
    void scoresUpdated(double s1, double s2, double coop);  // 점수가 바뀔 때
    void statusText(const QString& msg);                    // 상태 메시지 표시용

private:
    // 게임 설정할 떄 필요했덤 값
    static constexpr int EDGE_THR = 128; // 가이드 라인 감지 임계값 (0~255)
    static constexpr double SEARCH_RADIUS_FRAC = 0.05; // 근처 픽셀 검색 반경 비율
    static constexpr double TOL_FRAC = 0.02; // 그림 정확도 허용 오차 비율
    static constexpr double MIN_HIT_RATIO = 0.3; // 최소 적중률 (30%)
    static constexpr double ACC_WEIGHT = 0.6; // 정확도 가중치 (60%)
    static constexpr double LEN_WEIGHT = 0.2; // 선 길이 가중치 (20%)
    static constexpr double COV_WEIGHT = 0.2; // 값커버리지 가중치 (20%)

    int curStage = 1;  // 현재 스테이지 번호 (1부터 시작)

    // 현재 스테이지의 리소스 파일 경로를 반환
    QString currentStageRes() const;

    QImage guideOriginal;   // 원본 가이드 사진
    QImage guideScaled;     // 캔버스 크기에 맞게 조정된 가이드 이미지
    QImage maskScaled;      // 가이드 라인 마스크 (검은선만 추출한 이미지)
    int totalEdgePixels = 0; // 가이드 라인의 총 픽셀 개수

    // 가이드 이미지를 리소스에서 불러오기
    void loadGuideFromResource();

    // 캔버스 크기에 맞게 이미지들을 다시 만들기
    void rebuildScaledAndMask();

    // 가이드 이미지가 표시될 영역 계산
    QRectF guideTargetRect() const;

    // 위젯 좌표를 이미지 픽셀 좌표로 변환
    QPoint imagePointFromWidgetPoint(const QPointF &p) const;

    // 위젯 좌표를 정규화된 좌표(0~1)로 변환
    QPointF widgetToNorm(const QPointF& w) const;

    // 정규화된 좌표를 위젯 좌표로 변환
    QPointF normToWidget(const QPointF& n) const;

    QVector<QPointF> p1, p2;  // 플레이어1과 플레이어2가 그린 점들
    bool p1Active = false;    // 플레이어1이 현재 그리고 있는지 여부
    bool p2Active = false;    // 플레이어2가 현재 그리고 있는지 여부
    int localRole = 1;        // 내가 플레이어1인지 2인지 (1 또는 2)

    bool running = false;  // 게임이 실행중인지 여부
    QElapsedTimer timer;   // 게임 시간 측정용 타이머

    // 주어진 점들의 점수를 계산 (0~100점)
    double scoreFor(const QVector<QPointF> &pts) const;

    // 특정 픽셀 근처에 가이드 라인이 있는지 확인
    bool isEdgePixelNear(const QPoint& ip, int maxR, double* outDistPx = nullptr) const;

    // 그린 선들이 가이드를 얼마나 정확히 따라갔는지 마스크 이미지 생성
    QImage makeStrokeHitMask(double tolPx) const;

    // 전체 진행률 계산 (0~1 사이값)
    double progressByHitMask(double tolFrac) const;
};

#endif // DRAWINGCANVAS_H

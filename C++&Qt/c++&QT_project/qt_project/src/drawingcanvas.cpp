#include "../include/drawingcanvas.h"
#include <QPainter> // 화면에 그림을 그리기 위한 Qt 클래스
#include <QMouseEvent>// 마우스 클릭, 움직임 등의 이벤트 처리
#include <QtMath>  // 수학 계산 함수들 (qMin, qMax 등)
#include <climits>  // C++ 표준 라이브러리: INT_MAX 같은 상수들
#include <unordered_set>  // C++ 표준 라이브러리: 중복 제거용 집합 컨테이너
#include <cmath> // C++ 표준 라이브러리: sqrt, pow 등 수학 함수들

static inline QPointF mousePosF(const QMouseEvent* e) {
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)  // Qt6 이상이면
    return e->position();                   // Qt6 방식으로 마우스 위치 반환
#else                                       // Qt5면
    return e->localPos();                   // Qt5 방식으로 마우스 위치 반환
#endif
}

// DrawingCanvas 클래스의 생성자 객체가 만들어질 때 실행되는 함수
DrawingCanvas::DrawingCanvas(QWidget *parent)
    : QFrame(parent)  // 부모 클래스 QFrame의 생성자 호출
{
    // 마우스 추적 기능을 끔 (버튼을 누르지 않고 움직이는 것은 추적하지 않음)
    setMouseTracking(false);

    // 터치 스크린 이벤트도 받을 수 있도록 설정
    setAttribute(Qt::WA_AcceptTouchEvents, true);

    // 가이드 이미지를 리소스에서 불러오기
    loadGuideFromResource();

    // 화면 크기에 맞게 이미지 크기 조정 및 마스크 생성
    rebuildScaledAndMask();
}

// 현재 스테이지에 해당하는 이미지 파일 경로를 반환하는 함수
QString DrawingCanvas::currentStageRes() const {
    switch (curStage) {  // 현재 스테이지 번호에 따라
    case 1: return QString(":/img/stage1.png");  // 스테이지1 이미지 경로
    case 2: return QString(":/img/stage2.png");  // 스테이지2 이미지 경로
    case 3: return QString(":/img/stage3.png");  // 스테이지3 이미지 경로
    default: return QString(":/img/stage1.png"); // 기본값은 스테이지1
    }
}

// 스테이지를 변경하는 함수
void DrawingCanvas::setStage(int s) {
    // s를 1~3 사이 값으로 제한 (범위를 벗어나면 자동으로 1 또는 3으로 설정)
    int ns = qBound(1, s, 3);
    curStage = ns;  // 현재 스테이지 변수에 저장

    // 새로운 스테이지의 가이드 이미지 불러오기
    loadGuideFromResource();

    // 화면에 맞게 이미지 크기 조정
    rebuildScaledAndMask();

    // 이전에 그린 모든 선들 지우기
    resetStrokes();

    // UI에 상태 메시지 전송 (한글로 "스테이지 X 준비됨. [시작]을 누르세요.")
    emit statusText(u8"스테이지 " + QString::number(curStage) + u8" 준비됨. [시작]을 누르세요.");

    // 화면 다시 그리기 요청
    update();
}

// 리소스 파일에서 가이드 이미지를 불러오는 함수
void DrawingCanvas::loadGuideFromResource() {
    // 현재 스테이지에 해당하는 이미지 파일 불러오기
    QImage img(currentStageRes());

    // 이미지 로드가 실패했으면
    if (img.isNull()) {
        // 에러 메시지 전송하고 빈 이미지로 설정
        emit statusText(u8"리소스 로드 실패: " + currentStageRes());
        guideOriginal = QImage();
        return;
    }

    // 이미지를 ARGB32 형식으로 변환 (투명도와 색상 처리 최적화)
    guideOriginal = img.convertToFormat(QImage::Format_ARGB32_Premultiplied);

    // 성공 메시지 전송
    emit statusText(u8"스테이지 " + QString::number(curStage) + u8" 가이드 로드 완료");
}

// 화면 크기에 맞게 이미지 크기를 조정하고 윤곽선 마스크를 만드는 함수
void DrawingCanvas::rebuildScaledAndMask() {
    // 원본 이미지가 없으면 아무것도 하지 않음
    if (guideOriginal.isNull()) return;

    // 이미지가 표시될 영역 계산
    QRectF area = guideTargetRect();
    if (area.isEmpty()) return;  // 영역이 비어있으면 종료

    // 원본 이미지를 화면 크기에 맞게 조정
    // KeepAspectRatio = 비율 유지, SmoothTransformation = 부드럽게 크기 조정
    guideScaled = guideOriginal.scaled(area.size().toSize(),
                                       Qt::KeepAspectRatio,
                                       Qt::SmoothTransformation);

    // 윤곽선 감지를 위한 흑백 마스크 이미지 생성
    QImage g = guideScaled.convertToFormat(QImage::Format_ARGB32);  // RGB 형식으로 변환
    maskScaled = QImage(g.size(), QImage::Format_Grayscale8);       // 흑백 이미지 생성

    totalEdgePixels = 0; // 윤곽선 픽셀 개수 초기화

    // 이미지의 모든 픽셀을 검사하여 윤곽선 찾기
    for (int y = 0; y < g.height(); ++y) {
        // 한 줄씩 픽셀 데이터 읽기
        const QRgb* s = reinterpret_cast<const QRgb*>(g.constScanLine(y));
        uchar* d = maskScaled.scanLine(y);

        for (int x = 0; x < g.width(); ++x) {
            // RGB 값 추출
            const int r = qRed(s[x]);
            const int gg = qGreen(s[x]);
            const int b = qBlue(s[x]);

            // 밝기 계산
            const int lum = int(0.299*r + 0.587*gg + 0.114*b);

            // 어두운 픽셀이면 윤곽선으로 판단 (EDGE_THR=128보다 어두우면)
            const bool isEdge = (lum < EDGE_THR);

            // 마스크에 저장 윤곽선이면 255(흰색), 아니면 0(검은색)
            d[x] = isEdge ? 255 : 0;

            // 윤곽선 픽셀 개수 카운트
            if (isEdge) totalEdgePixels++;
        }
    }
}

// 가이드 이미지가 표시될 영역을 계산하는 함수
QRectF DrawingCanvas::guideTargetRect() const {
    const int m = 40; // 여백 크기 (40픽셀)
    // 전체 화면에서 여백만큼 줄인 영역 반환
    return rect().adjusted(m, m, -m, -m);
}

// 화면 좌표를 이미지 픽셀 좌표로 변환하는 함수
QPoint DrawingCanvas::imagePointFromWidgetPoint(const QPointF &p) const {
    // 가이드 이미지가 없으면 잘못된 좌표 반환
    if (guideScaled.isNull()) return QPoint(-1, -1);

    // 이미지가 표시되는 영역 가져오기
    QRectF area = guideTargetRect();

    // 이미지 크기
    QSizeF imgSz = guideScaled.size();

    // 이미지의 왼쪽 위 모서리 좌표 계산
    QPointF tl(area.center().x() - imgSz.width()/2.0,
               area.center().y() - imgSz.height()/2.0);

    // 화면 좌표에서 이미지 기준 상대 좌표로 변환
    QPointF rel = p - tl;

    // 실수를 정수로 반올림
    int x = int(std::round(rel.x()));
    int y = int(std::round(rel.y()));

    // 이미지 범위를 벗어나면 잘못된 좌표 반환
    if (x < 0 || y < 0 || x >= guideScaled.width() || y >= guideScaled.height())
        return QPoint(-1, -1);

    return QPoint(x, y);  // 유효한 이미지 좌표 반환
}

// 게임을 시작하는 함수
void DrawingCanvas::startGame() {
    resetStrokes();  // 이전에 그린 모든 선들 지우기
    running = true;  // 게임 실행 상태로 설정
    timer.restart(); // 타이머 시작 (게임 시간 측정용)

    // UI에 시작 메시지 전송
    emit statusText(u8"스테이지 " + QString::number(curStage) + u8" 시작!");
    update();  // 화면 다시 그리기
}

// 게임을 중지하는 함수
void DrawingCanvas::stopGame() {
    running = false;  // 게임 실행 상태를 중지로 설정

    // 현재 게임 상태의 점수들 계산
    Metrics m = computeMetrics();

    // UI에 점수 정보 전송
    emit scoresUpdated(m.s1, m.s2, m.coop);

    // UI에 결과 메시지 전송 (진행률을 소수점 1자리까지 표시)
    emit statusText(u8"채점 완료. 진행도 " + QString::number(m.progress*100.0,'f',1) + u8"%");
    update();  // 화면 다시 그리기
}

// 그린 모든 선들을 지우고 게임을 초기 상태로 되돌리는 함수
void DrawingCanvas::resetStrokes() {
    p1.clear();  // 플레이어1이 그린 점들 모두 삭제
    p2.clear();  // 플레이어2가 그린 점들 모두 삭제
    p1Active = false;  // 플레이어1 그리기 상태 해제
    p2Active = false;  // 플레이어2 그리기 상태 해제
    running = false;   // 게임 중지 상태로 설정
    update();  // 화면 다시 그리기
}

// 화면을 그리는 함수 - Qt에서 자동으로 호출됨
void DrawingCanvas::paintEvent(QPaintEvent*) {
    // QPainter 객체 생성 (이 객체로 화면에 그림을 그림)
    QPainter g(this);

    // 배경색을 연한 베이지색으로 칠하기
    g.fillRect(rect(), QColor("#fffdf5"));

    // 안티앨리어싱 켜기 (선이 부드럽게 보이도록)
    g.setRenderHint(QPainter::Antialiasing, true);

    // 가이드 이미지가 들어갈 영역 계산
    QRectF area = guideTargetRect();

    // 영역 테두리 그리기 (회색 점선, 둥근 모서리)
    g.setPen(QPen(QColor("#cfcfcf"), 2, Qt::DashLine));
    g.drawRoundedRect(area, 10, 10);

    // 가이드 이미지가 있으면 그리기
    if (!guideScaled.isNull()) {
        // 이미지 크기
        QSizeF imgSz = guideScaled.size();

        // 이미지를 중앙에 위치시키기 위한 왼쪽 위 좌표 계산
        QPointF tl(area.center().x() - imgSz.width()/2.0,
                   area.center().y() - imgSz.height()/2.0);

        // 투명도 설정 (1.0 = 완전 불투명)
        g.setOpacity(1.0);

        // 가이드 이미지 그리기
        g.drawImage(QRectF(tl, imgSz), guideScaled);
        g.setOpacity(1.0);  // 투명도 원래대로
    } else {
        // 이미지 로드 실패 시 에러 메시지 표시
        g.drawText(area, Qt::AlignCenter, "RESOURCE LOAD FAILED");
    }

    // 플레이어들이 그린 선을 그리기 위한 람다 함수 정의
    // auto = 자동으로 타입 추론 외부 변수들을 참조로 캡처
    auto drawStroke = [&](const QVector<QPointF>& pts, const QColor& c) {
        if (pts.size() < 2) return;  // 점이 2개 미만이면 선을 그릴 수 없음

        // 펜 설정: 색상 c, 두께 5픽셀, 실선, 둥근 끝점, 둥근 연결점
        g.setPen(QPen(c, 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

        // 모든 점들을 연결하여 선 그리기
        for (int i = 1; i < pts.size(); ++i) {
            g.drawLine(pts[i-1], pts[i]);
        }
    };

    // 플레이어1과 플레이어2의 선 그리기
    drawStroke(p1, QColor(40, 120, 255));   // P1 = 파란색
    drawStroke(p2, QColor(230, 60, 60));    // P2 = 빨간색
}

// 화면 좌표를 네트워크 전송용 정규화 좌표(0~1)로 변환
QPointF DrawingCanvas::widgetToNorm(const QPointF& w) const {
    QRectF r = rect();  // 현재 화면 크기
    // x, y 각각을 0~1 사이 값으로 변환
    return QPointF(w.x()/r.width(), w.y()/r.height());
}

// 네트워크에서 받은 정규화 좌표(0~1)를 화면 좌표로 변환
QPointF DrawingCanvas::normToWidget(const QPointF& n) const {
    QRectF r = rect();  // 현재 화면 크기
    // 정규화된 좌표를 실제 픽셀 좌표로 변환
    return QPointF(n.x()*r.width(), n.y()*r.height());
}

// 마우스 버튼을 눌렀을 때 호출되는 함수
void DrawingCanvas::mousePressEvent(QMouseEvent *e) {
    if (!running) return;  // 게임이 실행중이 아니면 무시

    // 마우스 위치 가져오기 (Qt 버전 호환)
    const QPointF w = mousePosF(e);

    // 내 역할에 따라 해당하는 플레이어의 그리기 시작
    if (localRole == 1) {
        p1Active = true;      // 플레이어1 그리기 활성화
        p1.push_back(w);      // 시작점 추가
    } else {
        p2Active = true;      // 플레이어2 그리기 활성화
        p2.push_back(w);      // 시작점 추가
    }

    // 네트워크로 내 입력을 상대방에게 전송
    emit localPress(widgetToNorm(w));
    update();  // 화면 다시 그리기
}

// 마우스를 움직일 때 호출되는 함수
void DrawingCanvas::mouseMoveEvent(QMouseEvent *e) {
    if (!running) return;  // 게임이 실행중이 아니면 무시

    // 마우스 위치 가져오기
    const QPointF w = mousePosF(e);

    // 내 역할에 따라 해당하는 플레이어가 현재 그리고 있으면 점 추가
    if (localRole == 1) {
        if (p1Active) p1.push_back(w);  // P1이 그리는 중이면 점 추가
    } else {
        if (p2Active) p2.push_back(w);  // P2가 그리는 중이면 점 추가
    }

    // 네트워크로 내 움직임을 상대방에게 전송
    emit localMove(widgetToNorm(w));
    update();  // 화면 다시 그리기
}

// 마우스 버튼을 놓았을 때 호출되는 함수
void DrawingCanvas::mouseReleaseEvent(QMouseEvent *e) {
    Q_UNUSED(e);  // 매개변수를 사용하지 않음을 명시

    // 모든 플레이어의 그리기 중지
    p1Active = false;
    p2Active = false;

    // 네트워크로 마우스 버튼 해제를 상대방에게 전송
    emit localRelease();
}

// 창 크기가 바뀔 때 호출되는 함수
void DrawingCanvas::resizeEvent(QResizeEvent *e) {
    // 부모 클래스의 리사이즈 이벤트 처리
    QFrame::resizeEvent(e);

    // 새로운 화면 크기에 맞게 이미지 다시 조정
    rebuildScaledAndMask();
    update();  // 화면 다시 그리기
}

// 네트워크에서 상대방이 마우스를 눌렀다는 신호를 받았을 때
void DrawingCanvas::netPress(int role, const QPointF& norm) {
    if (!running) return;  // 게임이 실행중이 아니면 무시

    // 정규화된 좌표를 화면 좌표로 변환
    const QPointF w = normToWidget(norm);

    // 해당 플레이어의 그리기 시작
    if (role == 1) {
        p1Active = true;
        p1.push_back(w);
    } else {
        p2Active = true;
        p2.push_back(w);
    }
    update();  // 화면 다시 그리기
}

// 네트워크에서 상대방이 마우스를 움직였다는 신호를 받았을 때
void DrawingCanvas::netMove(int role, const QPointF& norm) {
    if (!running) return;  // 게임이 실행중이 아니면 무시

    // 정규화된 좌표를 화면 좌표로 변환
    const QPointF w = normToWidget(norm);

    // 해당 플레이어가 그리는 중이면 점 추가
    if (role == 1) {
        if (p1Active) p1.push_back(w);
    } else {
        if (p2Active) p2.push_back(w);
    }
    update();  // 화면 다시 그리기
}

// 네트워크에서 상대방이 마우스 버튼을 놓았다는 신호를 받았을 때
void DrawingCanvas::netRelease(int role) {
    Q_UNUSED(role);  // 역할 정보는 사용하지 않음

    // 모든 플레이어의 그리기 중지
    p1Active = false;
    p2Active = false;
}

// 네트워크에서 상대방이 게임을 시작했다는 신호를 받았을 때
void DrawingCanvas::netStart() { startGame(); }

// 네트워크에서 상대방이 게임을 중지했다는 신호를 받았을 때
void DrawingCanvas::netStop() { stopGame(); }

// 네트워크에서 상대방이 리셋했다는 신호를 받았을 때
void DrawingCanvas::netReset() { resetStrokes(); }

// 특정 픽셀 근처에 가이드 라인(윤곽선)이 있는지 확인하는 함수
bool DrawingCanvas::isEdgePixelNear(const QPoint& ip, int maxR, double* outDistPx) const {
    // 좌표가 이미지 범위를 벗어나면 false
    if (ip.x() < 0 || ip.y() < 0 || ip.x() >= maskScaled.width() || ip.y() >= maskScaled.height())
        return false;

    // 해당 픽셀이 바로 윤곽선이면 거리 0으로 true 반환
    if (maskScaled.pixelColor(ip).value() > 0) {
        if (outDistPx) *outDistPx = 0.0;  // 거리 정보가 필요하면 0 저장
        return true;
    }

    // 주변 픽셀들을 검사하여 가장 가까운 윤곽선 픽셀 찾기
    int best2 = INT_MAX;  // 가장 가까운 거리의 제곱값

    // 반지름 maxR 내의 모든 픽셀 검사
    for (int dy = -maxR; dy <= maxR; ++dy) {
        int yy = ip.y() + dy;
        if (yy < 0 || yy >= maskScaled.height()) continue;  // 범위 벗어나면 건너뛰기

        for (int dx = -maxR; dx <= maxR; ++dx) {
            int xx = ip.x() + dx;
            if (xx < 0 || xx >= maskScaled.width()) continue;  // 범위 벗어나면 건너뛰기

            // 원형 범위 밖이면 건너뛰기
            if (dx*dx + dy*dy > maxR*maxR) continue;

            // 해당 픽셀이 윤곽선이면
            if (maskScaled.pixelColor(xx, yy).value() > 0) {
                int d2 = dx*dx + dy*dy;  // 거리의 제곱 계산
                if (d2 < best2) best2 = d2;  // 더 가까우면 업데이트
            }
        }
    }

    // 윤곽선을 찾지 못했으면 false
    if (best2 == INT_MAX) return false;

    // 거리 정보가 필요하면 실제 거리 계산해서 저장
    if (outDistPx) *outDistPx = std::sqrt((double)best2);
    return true;
}

// 플레이어가 그린 선들의 점수를 계산하는 함수 (0~100점)
double DrawingCanvas::scoreFor(const QVector<QPointF> &pts) const {
    // 점이 5개 미만이거나 가이드가 없으면 0점
    if (pts.size() < 5 || guideScaled.isNull()) return 0.0;

    // 기준 크기 계산 (이미지 가로세로 중 작은 값)
    const double ref = qMin(maskScaled.width(), maskScaled.height());

    // 검색 반지름 (기준 크기의 5%)
    const int maxR = qMax(1, int(ref * SEARCH_RADIUS_FRAC));

    // 허용 오차 거리 (기준 크기의 2%)
    const double tolPx = ref * TOL_FRAC;

    // 윤곽선 픽셀이 없으면 0점
    if (totalEdgePixels == 0) return 0.0;
    const int totalEdge = totalEdgePixels;

    // 성능을 위해 점들을 일정 간격으로 샘플링 (최대 900개까지만)
    const int step = qMax(1, pts.size() / 900);

    double sumPart = 0.0;  // 정확도 점수 합계
    int cnt = 0;           // 검사한 점의 개수
    int hits = 0;          // 윤곽선에 적중한 점의 개수

    // 중복 검사 방지를 위한 집합 (방문한 픽셀 좌표 저장)
    std::unordered_set<long long> visited;
    visited.reserve(4096);  // 메모리 미리 할당

    // 좌표를 하나의 긴 정수로 변환하는 람다 함수
    auto key = [](int x, int y) -> long long {
        return ((long long)y << 32) | (unsigned int)x;
    };

    // 샘플링된 점들을 검사
    for (int i = 0; i < pts.size(); i += step) {
        // 화면 좌표를 이미지 좌표로 변환
        const QPoint ip = imagePointFromWidgetPoint(pts[i]);
        if (ip.x() < 0) continue;  // 유효하지 않은 좌표면 건너뛰기

        double distPx = 1e9;  // 초기 거리를 매우 큰 값으로 설정

        // 근처에 윤곽선이 있는지 확인하고 거리 계산
        const bool near = isEdgePixelNear(ip, maxR, &distPx);

        double part = 0.0;  // 이 점의 정확도 점수

        if (near && distPx <= tolPx) {  // 허용 오차 내에 윤곽선이 있으면
            // 거리가 가까울수록 높은 점수 (1.2제곱으로 비선형 감소)
            part = 1.0 - std::pow(distPx / tolPx, 1.2);
            part = qMax(0.0, part);  // 음수 방지
            hits++;  // 적중 카운트

            // 방문한 픽셀 좌표 기록 (커버리지 계산용)
            int ex = qBound(0, ip.x(), maskScaled.width() - 1);
            int ey = qBound(0, ip.y(), maskScaled.height() - 1);
            visited.insert(key(ex, ey));
        }

        sumPart += part;  // 점수 누적
        cnt++;            // 검사한 점 개수 증가
    }

    if (cnt == 0) return 0.0;  // 검사한 점이 없으면 0점

    // 평균 정확도 계산
    double acc = sumPart / cnt;

    // 적중률이 최소 기준(30%) 미만이면 점수 감점
    const double hitRatio = (double)hits / (double)cnt;
    if (hitRatio < MIN_HIT_RATIO) {
        const double scale = (hitRatio / MIN_HIT_RATIO);
        acc *= std::pow(scale, 1.2);  // 비선형 감점
    }

    // 커버리지 점수: 전체 윤곽선 중 얼마나 많이 덮었는가
    const double coverage = qBound(0.0, (double)visited.size() / (double)totalEdge, 1.0);

    // 길이 팩터: 그린 선의 길이 (400개 점을 기준으로 정규화)
    const double lenFactor = qMin(1.0, pts.size() / 400.0);

    // 최종 점수 = 가중평균 (정확도 60% + 길이 20% + 커버리지 20%)
    const double score01 = ACC_WEIGHT*acc + LEN_WEIGHT*lenFactor + COV_WEIGHT*coverage;
    const double score = 100.0 * qBound(0.0, score01, 1.0);  // 0~100점으로 변환

    return qBound(0.0, score, 100.0);  // 범위 제한하여 반환
}

// 그린 선들을 굵기를 가진 마스크 이미지로 변환하는 함수
QImage DrawingCanvas::makeStrokeHitMask(double tolPx) const {
    // 가이드 이미지가 없으면 빈 이미지 반환
    if (maskScaled.isNull() || guideScaled.isNull())
        return QImage();

    // 마스크와 같은 크기의 흑백 이미지 생성
    QImage hit(maskScaled.size(), QImage::Format_Grayscale8);
    hit.fill(0);  // 검은색으로 초기화

    // 이미지에 그리기 위한 QPainter 생성
    QPainter p(&hit);
    p.setRenderHint(QPainter::Antialiasing, false);  // 안티앨리어싱 끔 (정확한 픽셀 처리)

    // 펜 두께 계산 (허용 오차의 2배 + 1)
    const int penW = qMax(1, int(std::lround(tolPx * 2.0 + 1.0)));

    // 흰색 펜으로 설정 (둥근 끝점과 연결점)
    QPen pen(Qt::white, penW, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    p.setPen(pen);

    // 점들을 연결하여 선을 그리는 람다 함수
    auto drawStrokeImg = [&](const QVector<QPointF>& pts) {
        if (pts.size() < 2) return;  // 점이 2개 미만이면 선을 그릴 수 없음

        QPoint prev = imagePointFromWidgetPoint(pts[0]);  // 이전 점
        for (int i = 1; i < pts.size(); ++i) {
            QPoint cur = imagePointFromWidgetPoint(pts[i]);  // 현재 점

            // 두 점 모두 유효하면 선 그리기
            if (prev.x() >= 0 && cur.x() >= 0) {
                p.drawLine(prev, cur);
            }
            prev = cur;  // 현재 점을 이전 점으로 설정
        }
    };

    // 두 플레이어의 선 모두 그리기
    drawStrokeImg(p1);
    drawStrokeImg(p2);

    p.end();  // 그리기 종료
    return hit;  // 완성된 마스크 이미지 반환
}

// 전체 진행률을 계산하는 함수 (0~1 사이 값)
double DrawingCanvas::progressByHitMask(double tolFrac) const {
    // 가이드가 없거나 윤곽선 픽셀이 없으면 0% 진행률
    if (guideScaled.isNull() || totalEdgePixels <= 0) return 0.0;

    // 기준 크기와 허용 오차 계산
    const double ref = qMin(maskScaled.width(), maskScaled.height());
    const double tolPx = ref * qMax(0.0, tolFrac);

    // 그린 선들의 히트 마스크 생성
    QImage hit = makeStrokeHitMask(tolPx);
    if (hit.isNull()) return 0.0;

    int covered = 0;  // 덮인 윤곽선 픽셀 개수
    const int W = maskScaled.width();
    const int H = maskScaled.height();

    // 모든 픽셀을 검사하여 덮인 윤곽선 픽셀 카운트
    for (int y = 0; y < H; ++y) {
        const uchar* edge = maskScaled.constScanLine(y);   // 윤곽선 마스크 한 줄
        const uchar* hln = hit.constScanLine(y);           // 히트 마스크 한 줄

        for (int x = 0; x < W; ++x) {
            // 윤곽선 픽셀이면서 히트 마스크도 있으면 덮인 것으로 카운트
            if (edge[x] > 0 && hln[x] > 0) covered++;
        }
    }

    // 진행률 = 덮인 픽셀 수 / 전체 윤곽선 픽셀 수
    double prog = (double)covered / (double)totalEdgePixels;
    return qBound(0.0, prog, 1.0);  // 0~1 사이로 제한
}

// 게임의 모든 점수와 진행률을 계산하는 함수
DrawingCanvas::Metrics DrawingCanvas::computeMetrics() const {
    Metrics m;  // 결과를 담을 구조체

    // 전체 진행률 계산 (허용 오차 2% 기준)
    m.progress = progressByHitMask(TOL_FRAC);

    // 각 플레이어의 개별 점수 계산
    m.s1 = scoreFor(p1);  // 플레이어1 점수
    m.s2 = scoreFor(p2);  // 플레이어2 점수

    // 협동 점수 계산
    const double better = std::max(m.s1, m.s2);  // 더 높은 점수
    const double worse = std::min(m.s1, m.s2);   // 더 낮은 점수

    // 협동 점수 = 높은점수*0.6 + 낮은점수*0.4 + 진행률*15
    double coop = 0.6 * better + 0.4 * worse + 15.0 * m.progress;
    m.coop = qBound(0.0, coop, 100.0);  // 0~100점으로 제한

    return m;  // 계산된 모든 점수 반환
}

// 현재 스테이지의 난이도에 따라 적절한 제한시간을 제안하는 함수
int DrawingCanvas::suggestDurationMs() const {
    // 윤곽선 픽셀이 없으면 기본값 60초 반환
    if (totalEdgePixels <= 0) return 60000;

    // 윤곽선 픽셀 개수의 제곱근을 기준으로 시간 계산
    const double c = std::sqrt((double)totalEdgePixels);
    int ms = int(80.0 * c);  // 튜닝된 공식 (80 * √픽셀수)

    // 최소 20초, 최대 180초로 제한
    ms = qBound(20000, ms, 180000);

    return ms;  // 밀리초 단위로 반환
}

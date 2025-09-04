#ifndef HW2_HPP
#define HW2_HPP

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <ctime>

using namespace std;


struct Point {
    int nX;  
    int nY;  
    

    Point() {
        nX = 0;
        nY = 0;
    }
    

    Point(int x, int y) {
        nX = x;
        nY = y;
    }
};


class DistanceCalculator {
private:
    Point* points;     
    int numPoints;      
    int minValue;       
    int maxValue;     

public:

    DistanceCalculator() {
        points = NULL;
        numPoints = 0;
        minValue = 0;
        maxValue = 0;
        srand(time(NULL));
    }
    
    ~DistanceCalculator() {
        if (points != NULL) {
            delete[] points;
        }
    }
    
    void inputData() {
        cout << "please define the number of points : ";
        cin >> numPoints;

        while (cin.fail()) {
            cin.clear();
            cin.ignore(1000, '\n');
            cout << "숫자만 입력해주세요." << endl;
            cout << "please define the number of points : ";
            cin >> numPoints;
        }
        
        while (numPoints < 2) {
            cout << "최소 2개의 점이 필요합니다." << endl;
            cout << "please define the number of points : ";
            cin >> numPoints;
        }
        
        cout << "please define minimum of coor. value : ";
        cin >> minValue;
        
        while (cin.fail()) {
            cin.clear();
            cin.ignore(1000, '\n');
            cout << "숫자만 입력해주세요" << endl;
            cout << "please define minimum of coor. value : ";
            cin >> minValue;
        }
        
        cout << "please define maximum of coor. value  : ";
        cin >> maxValue;
        
        while (cin.fail()) {
            cin.clear();
            cin.ignore(1000, '\n');
            cout << "숫자만 입력해주세요!" << endl;
            cout << "please define maximum of coor. value  : ";
            cin >> maxValue;
        }
        
        while (minValue >= maxValue) {
            cout << "최소값은 최대값보다 작아야 합니다." << endl;
            cout << "please define minimum of coor. value : ";
            cin >> minValue;
            cout << "please define maximum of coor. value  : ";
            cin >> maxValue;
        }
        
        points = new Point[numPoints];
    }
    
    void generatePoints() {
        cout << "\nGenerate Random points" << endl;
        
        for (int i = 0; i < numPoints; i++) {

            int x = minValue + rand() % (maxValue - minValue + 1);
            int y = minValue + rand() % (maxValue - minValue + 1);
            
            points[i] = Point(x, y);

            cout << "Point " << (i + 1) << ". nX=" << points[i].nX 
                 << " , nY=" << points[i].nY << endl;
        }
    }
    
    double calculateDistance(Point p1, Point p2) {
        int dx = p1.nX - p2.nX; 
        int dy = p1.nY - p2.nY;  
        
        // 루트(x1-x2)^2 + (y1-y2)^2)
        return sqrt(dx * dx + dy * dy);
    }
    

    void findminmaxDistance() {

        if (numPoints < 2) {
            cout << "거리를 계산할 점이 부족합니다." << endl;
            return;
        }
        
        double minDistance = 99999.0;  
        double maxDistance = 0.0;      

        int minIndex1 = 0, minIndex2 = 0;  
        int maxIndex1 = 0, maxIndex2 = 0;  

        for (int i = 0; i < numPoints; i++) {
            for (int j = i + 1; j < numPoints; j++) {
                double distance = calculateDistance(points[i], points[j]);

                if (distance < minDistance) {
                    minDistance = distance;
                    minIndex1 = i;
                    minIndex2 = j;
                }
                
                if (distance > maxDistance) {
                    maxDistance = distance;
                    maxIndex1 = i;
                    maxIndex2 = j;
                }
            }
        }

        cout << "\n--- Result ---" << endl;
        cout << "MinDist = " << fixed << setprecision(5) << minDistance << endl;
        cout << "Pair of Min Coor.(x,y): P1<" << points[minIndex1].nX << "," 
             << points[minIndex1].nY << "> & P2<" << points[minIndex2].nX 
             << "," << points[minIndex2].nY << ">" << endl;
        
        cout << "\nMaxDist = " << fixed << setprecision(4) << maxDistance << endl;
        cout << "Pair of Max Coor.(x,y): P1<" << points[maxIndex1].nX << "," 
             << points[maxIndex1].nY << "> & P2<" << points[maxIndex2].nX 
             << "," << points[maxIndex2].nY << ">" << endl;
        
        cout << "\n--- Completed ---" << endl;
    }
};

#endif

#ifndef HW1_HPP
#define HW1_HPP

#include <iostream>
#include <iomanip>
#include <string>
using namespace std;

class ArrayCalculator {
private:
    int* arr;
    int size;
    int max;
    int min;
    int sum;
    double average;

    bool isValidPInteger(const string& s) {
        if (s.length() == 0) 
        return false;

        int i = 0;
        while (i < (int)s.length()) {
            if (s[i] < '0' || s[i] > '9') 
            return false;
            i++;
        }

        i = 0;
        while (i < (int)s.length()) {
            if (s[i] != '0') 
            return true;
            i++;
        }
        return false; 
    }

    bool isValidInteger(const string& s) {
        if (s.length() == 0) 
        return false;

        int i = 0;
        if (s[0] == '-') {
            if (s.length() == 1) 
            return false;  
            i = 1;
        }

        while (i < (int)s.length()) {
            if (s[i] < '0' || s[i] > '9') 
            return false;
            i++;
        }
        return true;
    }

public:
    ArrayCalculator() {
        arr = NULL;
        size = 0;
        max = 0;
        min = 0;
        sum = 0;
        average = 0.0;
    }

    ~ArrayCalculator() {
        if (arr != NULL) {
            delete[] arr;
            arr = NULL;
        }
    }

    void inputArraySize() {
        string input;
        while (true) {
            cout << "몇 개의 원소를 할당하겠습니까? : ";
            getline(cin, input);

            if (!isValidPInteger(input)) {
                cout << "1 이상의 정수만 입력 가능합니다. 다시 입력해주세요." << endl;
                continue;
            }

            int number = 0;
            int idx = 0;
            while (idx < (int)input.length()) {
                number = number * 10 + (input[idx] - '0');
                idx++;
            }

            size = number;
            arr = new int[size];
            break;
        }
    }

    void inputArrayValues() {
        int i = 0;
        while (i < size) {
            cout << "정수형 데이터 입력: ";
            string input;
            getline(cin, input);

            if (!isValidInteger(input)) {
                cout << "정수(음수 포함)만 입력 가능합니다. 다시 입력하세요." << endl;
                continue;
            }

            int number = 0;
            int idx = (input[0] == '-') ? 1 : 0;
            bool negative = (input[0] == '-');

            while (idx < (int)input.length()) {
                number = number * 10 + (input[idx] - '0');
                idx++;
            }
            if (negative) number = -number;

            arr[i] = number;
            i++;
        }
    }

    void findMaximum() {
        max = arr[0];
        int i = 1;
        while (i < size) {
            if (arr[i] > max) max = arr[i];
            i++;
        }
    }

    void findMinimum() {
        min = arr[0];
        int i = 1;
        while (i < size) {
            if (arr[i] < min) min = arr[i];
            i++;
        }
    }

    void calculateTotalSum() {
        sum = 0;
        int i = 0;
        while (i < size) {
            sum = sum + arr[i];
            i++;
        }
    }

    void calculateAverageValue() {
        average = (double)sum / size;
    }

    void performCalculations() {
        findMaximum();
        findMinimum();
        calculateTotalSum();
        calculateAverageValue();
    }

    void displayResults() {
        performCalculations();
        cout << "최대값: " << max << endl;
        cout << "최소값: " << min << endl;
        cout << "전체합: " << sum << endl;
        cout << "평균: " << fixed << setprecision(6) << average << endl;
    }
};

#endif

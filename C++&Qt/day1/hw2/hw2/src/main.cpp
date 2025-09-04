#include <iostream>
#include "../include/hw2/hw2.hpp"
using namespace std;

int main() {
    DistanceCalculator calculator;
    calculator.inputData();
    calculator.generatePoints();
    calculator.findminmaxDistance();
}

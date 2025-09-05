#include "Points.hpp"
#include <iostream>
#include <ctime>   // rand() 시드용
using namespace std;

int main() {
    int N, xmin, xmax, ymin, ymax;

    cout << "점 개수 입력: ";
    cin >> N;
    cout << "x 범위 [xmin xmax]: ";
    cin >> xmin >> xmax;
    cout << "y 범위 [ymin ymax]: ";
    cin >> ymin >> ymax;

    srand((unsigned)time(0)); // 시드: 실행할 때마다 다른 난수 생성

    Points pts(N, xmin, xmax, ymin, ymax);
    pts.generate();
    pts.printAll();

    pts.minDistPair();
    pts.maxDistPair();

    return 0;
}

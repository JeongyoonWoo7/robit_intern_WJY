#include "Points.hpp"
#include <iostream>
using namespace std;

Points::Points(int num, int x1, int x2, int y1, int y2) {
    n = num;
    xmin = x1; xmax = x2;
    ymin = y1; ymax = y2;
    arr = new Point[n];
}

Points::~Points() {
    delete[] arr;
}

// 제곱거리 계산
int Points::dist2(Point a, Point b) {
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    return dx*dx + dy*dy;
}

// 랜덤 점 생성
void Points::generate() {
    for (int i = 0; i < n; i++) {
        arr[i].x = xmin + rand() % (xmax - xmin + 1);
        arr[i].y = ymin + rand() % (ymax - ymin + 1);
    }
}

// 점 전체 출력
void Points::printAll() {
    for (int i = 0; i < n; i++) {
        cout << "P" << i << " = (" << arr[i].x << "," << arr[i].y << ")\n";
    }
}

// 최소 거리 점쌍 찾기
void Points::minDistPair() {
    int minD = dist2(arr[0], arr[1]);
    int a = 0, b = 1;
    for (int i = 0; i < n; i++) {
        for (int j = i+1; j < n; j++) {
            int d = dist2(arr[i], arr[j]);
            if (d < minD) {
                minD = d;
                a = i; b = j;
            }
        }
    }
    cout << "최소 거리 점쌍: P" << a << "(" << arr[a].x << "," << arr[a].y << ")"
         << " - P" << b << "(" << arr[b].x << "," << arr[b].y << ")\n";
    cout << "제곱거리 = " << minD << "\n";
}

// 최대 거리 점쌍 찾기
void Points::maxDistPair() {
    int maxD = dist2(arr[0], arr[1]);
    int a = 0, b = 1;
    for (int i = 0; i < n; i++) {
        for (int j = i+1; j < n; j++) {
            int d = dist2(arr[i], arr[j]);
            if (d > maxD) {
                maxD = d;
                a = i; b = j;
            }
        }
    }
    cout << "최대 거리 점쌍: P" << a << "(" << arr[a].x << "," << arr[a].y << ")"
         << " - P" << b << "(" << arr[b].x << "," << arr[b].y << ")\n";
    cout << "제곱거리 = " << maxD << "\n";
}

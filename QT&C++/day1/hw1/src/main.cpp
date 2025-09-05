#include "Calculation.hpp"
#include <iostream>
using namespace std;

int main() {
    int n;
    cout << "갯수 입력: ";
    cin >> n;

    if (n <= 0) {
        cout << "크기는 1 이상의 정수여야 합니다.\n";
        return 0;
    }

    Calculation calc(n);
    calc.input();

    cout << "최댓값: " << calc.max() << '\n';
    cout << "최솟값: " << calc.min() << '\n';
    cout << "전체합: " << calc.sum() << '\n';
    cout << "평균  : " << calc.avg() << '\n';

    return 0;
}

#include "Calculation.hpp"
#include <iostream>

using std::cin;
using std::cout;

Calculation::Calculation(int n) : arr(nullptr), size(n) {
    if (n > 0) {
        arr = new int[size];
    } else {
        arr = nullptr;
        size = 0;
    }
}

Calculation::~Calculation() {
    delete[] arr;
}

void Calculation::input() {
    if (arr == nullptr) return;
    for (int i = 0; i < size; i++) {
        cout << i + 1 << "번째 값: ";
        cin >> arr[i];
    }
}

int Calculation::sum() const {
    if (arr == nullptr || size == 0) return 0;
    int total = 0;
    for (int i = 0; i < size; i++) total += arr[i];
    return total;
}

int Calculation::min() const {
    if (arr == nullptr || size == 0) return 0;
    int m = arr[0];
    for (int i = 1; i < size; i++) if (arr[i] < m) m = arr[i];
    return m;
}

int Calculation::max() const {
    if (arr == nullptr || size == 0) return 0;
    int m = arr[0];
    for (int i = 1; i < size; i++) if (arr[i] > m) m = arr[i];
    return m;
}

double Calculation::avg() const {
    if (arr == nullptr || size == 0) return 0.0;
    return static_cast<double>(sum()) / size;
}

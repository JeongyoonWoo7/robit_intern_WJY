#ifndef CALCULATION_H
#define CALCULATION_H

class Calculation {
public:
    explicit Calculation(int n);
    ~Calculation();

    void input();

    int    sum() const;
    int    min() const;
    int    max() const;
    double avg() const;

    // 과제 범위: 얕은복사 방지
    Calculation(const Calculation&) = delete;
    Calculation& operator=(const Calculation&) = delete;

private:
    int* arr;
    int  size;
};

#endif

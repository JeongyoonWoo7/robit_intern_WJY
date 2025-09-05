#ifndef POINTS_HPP
#define POINTS_HPP

struct Point {
    int x;
    int y;
};

class Points {
public:
    Points(int num, int x1, int x2, int y1, int y2);
    ~Points();

    void generate();
    void printAll();
    void minDistPair();
    void maxDistPair();

private:
    Point* arr;
    int n;
    int xmin, xmax, ymin, ymax;

    int dist2(Point a, Point b);
};

#endif

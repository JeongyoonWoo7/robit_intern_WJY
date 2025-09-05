#include "Game.hpp"
#include <iostream>


using std::cout;
using std::endl;

namespace {
    constexpr int kDamage = 10;     // 한 번 공격 시 데미지
}


Player::Player() : HP(100), MP(10), x(0), y(0) {}
Player::Player(int x_, int y_) : HP(100), MP(10), x(x_), y(y_) {}

void Player::Show_Status() {
    cout << "Player  HP:" << HP << "  MP:" << MP
         << "  Position:(" << x << "," << y << ")\n";
}

void Player::X_move(int move) { x += move; }
void Player::Y_move(int move) { y += move; }

void Player::Attack(Monster &target) {
    if (MP <= 0) {
        cout << "MP 부족!\n";
        cout << "게임 종료\n";

        exit(0); // 종료
    }
    MP -=1; // 공격 실패해도 마나 감소
    

    if (x == target.x && y == target.y) {
        MP -= 1;
        int remain = target.Be_Attacked();
        cout << "공격 성공!\n";
        cout << "남은 체력: " << remain << "\n";
    } else {
        cout << "공격 실패!\n";
    }
}


Monster::Monster() : HP(50), x(5), y(4) {}
Monster::Monster(int x_, int y_, int HP_) : HP(HP_), x(x_), y(y_) {}

int Monster::Be_Attacked() {
    HP -= kDamage;
    if (HP <=0) HP=0;
    return HP;
}

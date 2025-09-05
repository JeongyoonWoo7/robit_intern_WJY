#include "Game.hpp"
#include <iostream>

using namespace std;

int main() {
    Player player(0, 0);
    Monster monster(5, 4, 50);

    cout << "Type Command: U/D/L/R/S(상태)/A(공격)\n";
    
    while (true) {
        cout << "Type Command (U/D/L/R/S/A): ";
        char cmd;
        if (!(cin >> cmd)) break;

        switch (cmd) {
            case 'U': case 'u': player.Y_move(+1); break;
            case 'D': case 'd': player.Y_move(-1); break;
            case 'L': case 'l': player.X_move(-1); break;
            case 'R': case 'r': player.X_move(+1); break;
            case 'S': case 's': player.Show_Status(); break;
            case 'A': case 'a':
                if (player.MP < 0) { 
                    cout << "MP가 없어 게임 종료.\n"; 
                    return 0; 
                }
                player.Attack(monster);
                break;

            default: cout << "Wrong Command!\n"; break;
        }

       
        if (monster.HP <= 0) {
            cout << "남은 체력: " << monster.HP << "\n";
            cout << "Monster Die!\n";
            return 0;
        }
    }
    return 0;
}

#include <Arduino.h>
#include <SmartCar.cpp>
#include <speed.cpp>
#include <array.cpp> // custom array class
#include <pair.cpp>

const int maxn = 4 + 1; // grid size

// empty = 0
// barricade = 1
// goal = 2

SmartCar car;
bool grid[maxn][maxn];
bool vis[maxn][maxn];
int dx[4] = {1, 1, -1, -1};
int dy[4] = {1, -1, 1, -1};

pair<array, array> dfs(int x, int y, array xCoords, array yCoords){
    if (x < 1 || x > maxn) return; // out of x bounds
    if (y < 1 || y > maxn) return; // out of y bounds
    if (grid[x][y] == 1) return; // barricade

    vis[x][y] = true;
    xCoords.push_back(x);
    yCoords.push_back(y);
    if (grid[x][y] == 2) { // goal
        return pair<array, array>(xCoords, yCoords);
    }

    for (int i = 0; i < 4; i ++){
        int nx = x + dx[i];
        int ny = y + dy[i];
        return dfs(nx, ny, xCoords, yCoords);
    }
}

void setup(){

    calcSpeed(2, 30);    
    dfs(1, 1, array(maxn), array(maxn));


}

void loop(){

}
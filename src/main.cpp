#include <Arduino.h>
#include <SmartCar.cpp>
#include <speed.cpp>
#include <vector.cpp> // custom array class
#include <pair.cpp>

#define vii vector<pair<int, int>>
#define pii pair<int, int>

const int maxn = 4; // grid size

// empty = 0
// barricade = 1
// goal = 2

SmartCar car;
int grid[maxn][maxn] = {
    {0, 0, 0, 0},
    {1, 1, 1, 0},
    {0, 0, 0, 0},
    {2, 1, 1, 1}};
bool vis[maxn][maxn];
int dx[4] = {1, -1, 0, 0};
int dy[4] = {0, 0, 1, -1};

void dfs(pii coord){

    int y = coord.first;
    int x = coord.second;

    if (vis[y][x]) return;
    if (grid[y][x] == 1) return;
    vis[y][x] = true;

    if (grid[y][x] == 2) {
        Serial.println("FOUND!");
        Serial.println("Exit: ( x: " + String(x) + ", y: " + String(y) + ")");
    }

    for (int i = 0; i < 4; i ++){
        int ny = y + dy[i];
        int nx = x + dx[i];

        if ( nx >= 0 && nx < maxn && ny >= 0 && ny < maxn){
            dfs(pii(ny, nx));
        }
    }

}

void setup()
{
    Serial.begin(9600);
    Serial.println("Started DFS");
    dfs(pii(0, 0));
    Serial.println("Ended DFS");
}

void loop()
{
}
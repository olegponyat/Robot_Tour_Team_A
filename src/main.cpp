#include <Arduino.h>
#include <SmartCar.cpp>
#include <speed.cpp>
#include <vector.cpp> // custom array class
#include <pair.cpp>

#define vii vector<pair<int, int>>
#define pii pair<int, int>

const int maxn = 7; // grid size

// empty = 0
// barricade = 1
// goal = 2

SmartCar car;

int grid[maxn][maxn] = {

// s = start, l = grid line
// 0 = grid, 1 = barrier, 2 = goal,
// 3 = intersection (impossible to get to, intersection between grid lines)

//   s  l     l     l
    {0, 0, 0, 0, 0, 0, 0},
    {1, 3, 1, 3, 1, 3, 0}, // <- l
    {0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 3, 1, 3, 1}, // <- l
    {0, 0, 0, 0, 0, 0, 0},
    {1, 3, 0, 3, 1, 3, 1}, // <- l
    {0, 0, 0, 0, 0, 0, 2}

};

bool vis[maxn][maxn];
int dx[4] = {1, -1, 0, 0};
int dy[4] = {0, 0, 1, -1};

void visualize(){
    Serial.println("=== VISUALIZATION ===");
    for (int i = 0; i < maxn; i ++){
        for (int j = 0; j < maxn; j ++){
            if (grid[i][j] == 0) Serial.print("░░");
            else if (grid[i][j] == 1 || grid[i][j] == 3) Serial.print("██");
            else if (grid[i][j] == 2) Serial.print("⭐");
        }
        Serial.println();
    }
    Serial.println();
}

bool dfs(pii coord, vii &paths){

    int y = coord.first;
    int x = coord.second;

    if (vis[y][x]) return false;
    if (grid[y][x] == 1 || grid[y][x] == 3) return false; // barrier or impossible
    vis[y][x] = true;

    if (grid[y][x] == 2) {
        Serial.println("Found goal at: ( x: " + String(x) + ", y: " + String(y) + ")");
        if (paths.length() == 0){
            return true;
        }
    }

    for (int i = 0; i < 4; i ++){
        int ny = y + dy[i];
        int nx = x + dx[i];

        if ( nx >= 0 && nx < maxn && ny >= 0 && ny < maxn){
            bool is_solution = dfs(pii(ny, nx), paths);
            if (is_solution){
                paths.push_back(pii(ny, nx));
                return true;
            } else{
                paths.clear();
            }
        }
    }

    return false;

}

void setup()
{
    Serial.begin(9600);
    visualize();

    Serial.println("Started DFS");
    vii result = vii();
    dfs(pii(0, 0), result);
    result.reverse(); // results are from goal to start, need to reverse
    Serial.println("\nSteps (y, x): ");
    for (int i = 0; i < result.length(); i ++){
        Serial.print("( " + String(result.get(i).first) + ", " + String(result.get(i).second) + ") -> ");
    }

}

void loop()
{
}
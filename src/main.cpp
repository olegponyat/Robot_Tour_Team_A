#include <Arduino.h>
#include <SmartCar.cpp>
#include <speed.cpp>
#include <vector.cpp> // custom array class
#include <pair.cpp>

#define vii vector<pair<int, int>>
#define pii pair<int, int>

/*SETTINGS*/
const float targetTime = 30;
const int maxn = 7; // grid size
const pii start = pii(6, 0);

/*DEBUG*/
bool debug_move = true;
// empty = 0
// barricade = 1
// goal = 2


int grid[maxn][maxn] = {

/*
s = start, l = grid line
0 = grid, 1 = barrier, 2 = goal,
3 = intersection (impossible to get to, intersection between grid lines)

Directions:
N,E,S,W

N = towards y = 0
W = towards x = 0
*/

//   s  l     l     l
    {2, 0, 0, 1, 0, 0, 0},
    {0, 3, 0, 3, 0, 3, 0}, // <- l
    {0, 0, 0, 1, 0, 0, 0},
    {0, 3, 1, 3, 1, 3, 1}, // <- l
    {0, 0, 0, 0, 0, 0, 0},
    {1, 3, 1, 3, 1, 3, 0}, // <- l
    {0, 0, 0, 0, 0, 0, 0}

};

bool graphSetupError = false;
bool vis[maxn][maxn];
int dx[4] = {1, -1, 0, 0};
int dy[4] = {0, 0, 1, -1};

SmartCar car;

/*
Grid lines are odd numbers:
- ex (for y): 1, 3, 5...
- ex (for x): 1, 3, 5...
*/
bool isGridLine(int index){
    return index%2 != 0;
}

void visualize(){
    Serial.println("\n[VISUALIZATION]\n");

    Serial.println("🚩= start");
    Serial.println("⭐ = end");
    Serial.println("🛑 = intersection (impossible)");
    Serial.println("░░ = grid line");
    Serial.println("▒▒ = empty");
    Serial.println("██ = barrier");
    Serial.println();
    for (int y = 0; y < maxn; y ++){
        for (int x = 0; x < maxn; x ++){

            if (y == start.first && x == start.second) { // start
                Serial.print("🚩");
            }
            else if (grid[y][x] == 2) { // end
                Serial.print("⭐");
            }
            // if it is a grid line, and it is empty. else it will be handled as barrier.
            else if ( (isGridLine(y) || isGridLine(x)) && grid[y][x] == 0){ 
                Serial.print("░░");
            }
            else if (grid[y][x] == 0) {
                Serial.print("▒▒");
            }
            else if (grid[y][x] == 1) {
                Serial.print("██");
            }
            else if (grid[y][x] == 3){
                Serial.print("🛑");
            }

            // barrier on an empty block, which is not legal
            if ( (!isGridLine(y) && !isGridLine(x)) && grid[y][x] == 1){
                graphSetupError = true;
            } 

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

void turnRight(int repeats=1){
    for (int i = 0; i < repeats; i ++){
        car.moveRight(100);
        delay(500);
        car.stop();
        delay(200);
    }
    car.stop();
}

void executePath(float analogSpeed, float msPerMove, vii points){

    Serial.println("Analog Speed: " + String(analogSpeed));

    pii currentPoint = start;
    int currentAngle = 0;
    bool prevChangeInY = true;

    for (int i = 0; i < points.length(); i ++){

        pii nextPos = points.get(i);

        int dy = nextPos.first - currentPoint.first;
        int dx = nextPos.second - currentPoint.second;
        int turns = currentAngle / 90;

        if (debug_move){
            Serial.println("[change (y,x)]( " + String(dy) + ", " + String(dx));
        }

        if (dy < 0){ // North (up)
            if (debug_move) Serial.println("Move North");
            if (prevChangeInY != true) turnRight(turns);
            car.moveFoward(analogSpeed);
            delay(msPerMove);
            prevChangeInY = true;
        }
        else if (dy > 0){ // South (down)
            if (debug_move) Serial.println("Move South");
            if (prevChangeInY != true) turnRight(turns);
            car.moveFoward(analogSpeed);
            delay(msPerMove);
            prevChangeInY = true;
        }
        else if (dx > 0){ // East (right)
            if (debug_move) Serial.println("Move East");
            if (prevChangeInY != false) {
                turnRight(1);
            }
            currentAngle += 90;
            car.moveFoward(analogSpeed);
            delay(msPerMove);
            prevChangeInY = false;
        }
        else if (dx < 0){ // West (left)
            if (debug_move) Serial.println("Move West");
            if (prevChangeInY != false) {
                turnRight(3);
            }
            currentAngle -= 90;
            car.moveFoward(analogSpeed);
            delay(msPerMove);
            prevChangeInY = false;
        }

        car.stop();

        currentAngle = 0;
        currentPoint = nextPos;
    }

}

void setup()
{
    Serial.begin(9600);
    visualize();

    if (graphSetupError){
        Serial.println("GRAPH SETUP ERROR. POSSIBLE ERRORS: ILLEGAL BARRIER SPOT.");
        return;
    }

    Serial.println("Started DFS");
    vii result = vii();
    dfs(start, result);
    result.reverse(); // results are from goal to start, need to reverse

    if (result.length() == 0) {
        Serial.println("No solution");
        return;
    }

    int moves = 0;
    vii points;

    Serial.println("\nSteps (y, x): ");
    for (int i = 0; i < result.length(); i ++){
        int y = result.get(i).first;
        int x = result.get(i).second;

        if (isGridLine(y) || isGridLine(x)){
            Serial.print("[grid]( " + String(y) + ", " + String(x) + ") -> ");
        }else{
            Serial.print("[point]( " + String(y) + ", " + String(x) + ") -> ");

            points.push_back(result.get(i));
            moves++;
        }

    }

    float secondsPerMove = targetTime / (float) moves;

    Serial.println("\n\n[Algorithm Summary]\n");
    Serial.println("Total Moves: " + String(moves));

    float analogSpeed = calcSpeed(moves, targetTime);
    executePath(analogSpeed, secondsPerMove * 1000, points);

}

void loop()
{
}
#include <Arduino.h>
#include <SmartCar.cpp>
#include <speed.cpp>
#include <vector.cpp> // custom array class
#include <pair.cpp>
#include <system.cpp>

#define vii vector<pair<int, int>>
#define pii pair<int, int>

/*SETTINGS*/
const float targetTime = 20;
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

// void visualize() {
//     Serial.println(F("\n[VISUALIZATION]\n"));

//     Serial.println(F("S = start, E = end, X = intersection, , = grid line, . = empty, # = barrier"));
//     Serial.println();
//     String line;
//     for (byte y = 0; y < maxn; y++) {
//         line = "";  // Reset line string for each row
//         for (byte x = 0; x < maxn; x++) {
//             if (y == start.first && x == start.second) { // start
//                 line += "S";
//             }
//             else if (grid[y][x] == 2) { // end
//                 line += "E";
//             }
//             else if ((isGridLine(y) || isGridLine(x)) && grid[y][x] == 0) { 
//                 line += ",";
//             }
//             else if (grid[y][x] == 0) {
//                 line += ".";
//             }
//             else if (grid[y][x] == 1) {
//                 line += "#";
//             }
//             else if (grid[y][x] == 3) {
//                 line += "X";
//             }

//             if ((!isGridLine(y) && !isGridLine(x)) && grid[y][x] == 1) {
//                 graphSetupError = true;
//             } 
//         }
//         Serial.println(line);
//     }
//     Serial.println();
// }

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
                // only if it isnt a grid line we count it as a move
                if (!isGridLine(ny) && !isGridLine(nx)) paths.push_back(pii(ny, nx));
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

void executePath(float analogSpeed, float msPerMove, vii &points){

    Serial.println("Analog Speed: " + String(analogSpeed));

    pii currentPoint = start;
    int dir = 0; //  0 = N, 1 = E, 2 = S , 3 = W

    for (int i = 0; i < points.length(); i ++){

        pii nextPos = points.get(i);

        int dy = nextPos.first - currentPoint.first;
        int dx = nextPos.second - currentPoint.second;

        if (debug_move){
            Serial.println("[change (y,x)]( " + String(dy) + ", " + String(dx) + ")");
        }
        
        if (dy < 0){ // North (up)
            if (debug_move) Serial.println("Move North");
            
            if (dir == 1) car.turnLeft(analogSpeed);
            else if (dir == 2) car.turnRight(analogSpeed);

            dir = 0;

            car.moveFoward(analogSpeed);
            delay(msPerMove);
        }
        else if (dy > 0){ // South (down)
            if (debug_move) Serial.println("Move South");

            if (dir == 1) car.turnLeft(analogSpeed);
            else if (dir == 2) car.turnRight(analogSpeed); 

            dir = 0;       

            car.moveBackward(analogSpeed);
            delay(msPerMove);
        }
        else if (dx > 0){ // East (right)
            if (debug_move) Serial.println("Move East");
        
            if (dir == 0) car.turnRight(analogSpeed);
            else if (dir == 2) {
                car.turnRight(analogSpeed);
                car.turnRight(analogSpeed);
            }

            dir = 1;

            car.moveFoward(analogSpeed);
            delay(msPerMove);
        }
        else if (dx < 0){ // West (left)
            if (debug_move) Serial.println("Move West");
            
            if (dir == 0) car.turnLeft(analogSpeed);
            else if (dir == 1) {
                car.turnLeft(analogSpeed);
                car.turnLeft(analogSpeed);
            }

            dir = 2;

            car.moveFoward(analogSpeed);
            delay(msPerMove);
        }

        car.stop();

        currentPoint = nextPos;
    }

}

void setup()
{
    Serial.begin(9600);
    getFreeRAMSpace();
    // visualize();

    if (graphSetupError){
        Serial.println("GRAPH SETUP ERROR. POSSIBLE ERRORS: ILLEGAL BARRIER SPOT.");
        return;
    }

    Serial.println("Started DFS");
    vii* result = new vii();
    dfs(start, *result);
    result->reverse(); // results are from goal to start, need to reverse
    getFreeRAMSpace();

    if (result->length() == 0) {
        Serial.println("No solution");
        return;
    }else Serial.println("Found valid solution");

    int moves = result->length();
    float analogSpeed = calcSpeed(moves, targetTime);
    float secondsPerMove = targetTime / (float) moves;

    Serial.println("\n\n[Algorithm Summary]");
    Serial.println("Total Moves: " + String(moves));
    Serial.println("Analog Speed: " + String(analogSpeed));
    Serial.println("Seconds Per Move: " + String(secondsPerMove));

    getFreeRAMSpace();
    Serial.print("Initiating car...");
    car.init();
    Serial.println("Successfully initiated car");
    executePath(analogSpeed, secondsPerMove * 1000, *result);
}

void loop(){}
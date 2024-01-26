#include <Arduino.h>
#include <SmartCar.cpp>
#include <speed.cpp>
#include <system.cpp>
// #include <pair.cpp>
#include <StandardCplusplus.h>
#include <map>
#include <queue>
#include <vector>

typedef std::pair<int, int> pii;
typedef std::vector<pii> vii;

/*SETTINGS*/
const bool useDistance = false;
const float delayBetweenMovesMS = 50; // delay between turn or move forward/backwards
const float targetTime = 20;
const int maxn = 7; // grid size
const pii start = pii(6, 2); // y, x
const float moveDistance = 0.5; // in meters

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
4 = bonus points

Directions:
N,E,S,W

N = towards y = 0
W = towards x = 0
*/

//   s  l     l     l
    {0, 1, 0, 0, 2, 1, 0},
    {0, 3, 0, 3, 0, 3, 0}, // <- l
    {4, 0, 0, 1, 0, 0, 4},
    {1, 3, 0, 3, 0, 3, 1}, // <- l
    {0, 0, 0, 0, 0, 0, 0},
    {1, 3, 0, 3, 4, 3, 0}, // <- l
    {0, 1, 0, 0, 0, 1, 0}

};

bool graphSetupError = false;
bool vis[maxn][maxn];
int dx[4] = {2, -2, 0, 0};
int dy[4] = {0, 0, 2, -2};
int gx[4] = {1, -1, 0, 0};
int gy[4] = {0, 0, 1, -1};

SmartCar car;

/*
Grid lines are odd numbers:
- ex (for y): 1, 3, 5...
- ex (for x): 1, 3, 5...
*/

void clearVis(){
    for (int i = 0; i < maxn; i ++)
        for (int j = 0; j < maxn; j ++)
            vis[i][j] = false;
}

vii BFS(pii start, bool &solved){
    clearVis();
    std::queue<pii> q;
    std::map<pii, pii> parent;
    
    vis[start.first][start.second] = true;
    q.push(start);
    
    while (!q.empty()){
        pii current = q.front(); q.pop();
        int y = current.first, x = current.second;
        vis[y][x] = true;
        
        if (grid[y][x] == 4 || grid[y][x] == 2){
            if (grid[y][x] == 2) solved = true;
            vii path;
            while (current != start){
                path.push_back(current);
                current = parent[current];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }
        
        for (int i = 0 ; i < 4; i ++){
            int ny = y + dy[i];
            int nx = x + dx[i];
            int ngy = y + gy[i];
            int ngx = x + gx[i];
            
            if ( nx >= 0 && nx < maxn && ny >= 0 && ny < maxn){
                if (grid[ngy][ngx] != 0) continue;
                if (vis[ny][nx]) continue;
                pii neighbor = std::make_pair(ny, nx);
                vis[ny][nx] = true;
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
        
    }
    
    return vii();
}

void executePath(float analogSpeed, float msPerMove, vii &points){

    Serial.println("Analog Speed: " + String(analogSpeed));

    pii currentPoint = start;
    int dir = 0; //  0 = N, 1 = E, 2 = S , 3 = W

    for (size_t i = 0; i < points.size(); i ++){

        pii nextPos = points[i];

        int dy = nextPos.first - currentPoint.first;
        int dx = nextPos.second - currentPoint.second;

        if (debug_move){
            Serial.println("[change (y,x)]( " + String(dy) + ", " + String(dx) + ")");
        }
        
        if (dy < 0){ // North (up)
            if (debug_move) Serial.println("Move North");
            
            if (dir == 1) car.turnLeft(analogSpeed);
            else if (dir == 2) {
                car.turnLeft(analogSpeed);
                car.turnLeft(analogSpeed);
            }
            else if (dir == 3) car.turnRight(analogSpeed);

            if (dir != 0) delay(delayBetweenMovesMS);

            dir = 0;

            if (useDistance) car.moveForwardDistance(analogSpeed, moveDistance);
            else car.moveForwardForSeconds(analogSpeed, msPerMove);

            car.adjust(analogSpeed);
        }
        else if (dy > 0){ // South (down)
            if (debug_move) Serial.println("Move South");

            if (dir == 0) {
                car.turnRight(analogSpeed);
                car.turnRight(analogSpeed);
            }
            else if (dir == 1) car.turnRight(analogSpeed); 
            else if (dir == 3) car.turnLeft(analogSpeed); 

            if (dir != 2) delay(delayBetweenMovesMS);

            dir = 2;       

            if (useDistance) car.moveForwardDistance(analogSpeed, moveDistance);
            else car.moveForwardForSeconds(analogSpeed, msPerMove);

            car.adjust(analogSpeed);
        }
        else if (dx > 0){ // East (right)
            if (debug_move) Serial.println("Move East");
        
            if (dir == 0) car.turnRight(analogSpeed);
            else if (dir == 2) car.turnLeft(analogSpeed);
            else if (dir == 3) {
                car.turnRight(analogSpeed);
                car.turnRight(analogSpeed);
            }

            if (dir != 1) delay(delayBetweenMovesMS);

            dir = 1;

            if (useDistance) car.moveForwardDistance(analogSpeed, moveDistance);
            else car.moveForwardForSeconds(analogSpeed, msPerMove);
        }
        else if (dx < 0){ // West (left)
            if (debug_move) Serial.println("Move West");
            
            if (dir == 0) car.turnLeft(analogSpeed);
            else if (dir == 1) {
                car.turnLeft(analogSpeed);
                car.turnLeft(analogSpeed);
            }else if (dir == 2) car.turnRight(analogSpeed);

            if (dir != 3) delay(delayBetweenMovesMS);

            dir = 3;

            if (useDistance) car.moveForwardDistance(analogSpeed, moveDistance);
            else car.moveForwardForSeconds(analogSpeed, msPerMove);
        }

        currentPoint = nextPos;
        car.adjust(analogSpeed);
        car.stop();
        delay(delayBetweenMovesMS);
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

    Serial.println("Started BFS");

    bool reachedGoal = false;
    vii result;
    
    pii nextStart = start;
    
    while (!reachedGoal){
        vii path = BFS(nextStart, reachedGoal);
        
        if (path.size() == 0){
            Serial.println("NO SOLUTION");
            return;
        }
        
        nextStart = path[path.size()-1];
        result.insert(result.end(), path.begin(), path.end() - 1); 
            // path.end() - 1 prevents adding a repeated point, bc the current endpoint is the new startpoint        
            
        if (reachedGoal) result.push_back(nextStart); // adds back the endpoint if it is solution
        
        grid[nextStart.first][nextStart.second] = 0; // removes the bonus points 
    }

    Serial.println("Found valid solution");

    if (debug_move){
        for (size_t i = 0; i < result.size(); i ++){
            Serial.print("(" + String(result[i].first) + "," + String(result[i].second) + ") => ");
        }
        Serial.println();
    }

    int moves = result.size();
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

    // Initial drive into the maze
    car.moveForwardForSeconds(analogSpeed, secondsPerMove * 1000 * 0.75);

    executePath(analogSpeed, secondsPerMove * 1000, result);

    // car.init();
    // car.moveForwardDistance(100,0.5);
}

void loop(){}
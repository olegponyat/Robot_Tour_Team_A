

float calcSpeed(int moves, float total_time){
    // 0.224*x+0.135
    float move_dist = 0.5; // one block in meters
    float total_dist = move_dist * moves;
    float speed = total_dist/total_time;
    float analog =(speed-0.135)/0.224;
    return analog;
}
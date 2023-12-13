float calcSpeed(int moves, float total_time){
    // 0.224*x+0.135
    //0.00313X - 0.00248
    float move_dist = 0.4; // one block in meters
    float total_dist = move_dist * moves;
    float speed = total_dist/total_time;
    float analog =(speed+0.00248)/0.00313;

    if(analog > 255){
        return 255;
    }
    else{
        return analog;
    }
}
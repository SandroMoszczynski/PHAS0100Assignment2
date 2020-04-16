#include "sfmPedestrian.h"

namespace sfm{

Pedestrians::Pedestrians(pos2d _origin,pos2d _destination, dir2d _velocity,pos2d _current_position,double _speed,double _rest_time){

    origin = _origin;
    destination = _destination;
    velocity = _velocity;
    current_position = _current_position;
    speed = _speed;
    rest_time = _rest_time;

}

pos2d &Pedestrians::Return_Origin(){
    return origin;
};

pos2d &Pedestrians::Return_Destination(){
    return destination;
};

dir2d &Pedestrians::Return_Velocity(){
    return velocity;
};

pos2d &Pedestrians::Return_Current_Position(){
    return current_position;
};

double &Pedestrians::Return_Speed(){
    return speed;
};

double &Pedestrians::Return_Rest_time(){
    return rest_time;
};

}
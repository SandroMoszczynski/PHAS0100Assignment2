#include "sfmPedestrian.h"

namespace sfm{

Pedestrians::Pedestrians(vec2d _origin,vec2d _destination, vec2d _velocity,vec2d _current_position,double _speed,double _rest_time){

    origin = _origin;
    destination = _destination;
    velocity = _velocity;
    current_position = _current_position;
    speed = _speed;
    rest_time = _rest_time;

}

vec2d &Pedestrians::Return_Origin(){
    return origin;
};

vec2d &Pedestrians::Return_Destination(){
    return destination;
};

vec2d &Pedestrians::Return_Velocity(){
    return velocity;
};

vec2d &Pedestrians::Return_Current_Position(){
    return current_position;
};

double &Pedestrians::Return_Speed(){
    return speed;
};

double &Pedestrians::Return_Rest_time(){
    return rest_time;
};

}
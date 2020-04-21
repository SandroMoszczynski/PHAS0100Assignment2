#include "sfmPedestrian.h"

namespace sfm{

Pedestrian::Pedestrian(pos2d _origin,pos2d _destination, dir2d _velocity,pos2d _current_position,double _speed,double _rest_time){

    origin = _origin;
    destination = _destination;
    velocity = _velocity;
    current_position = _current_position;
    speed = _speed;
    rest_time = _rest_time;
}

pos2d &Pedestrian::Return_Origin(){
    return origin;
};

pos2d &Pedestrian::Return_Destination(){
    return destination;
};

dir2d &Pedestrian::Return_Velocity(){
    return velocity;
};

pos2d &Pedestrian::Return_Current_Position(){
    return current_position;
};

double &Pedestrian::Return_Speed(){
    return speed;
};

double &Pedestrian::Return_Rest_time(){
    return rest_time;
};

dir2d &Pedestrian::Update_Velocity(dir2d new_velocity){
    velocity = new_velocity;    
};

pos2d &Pedestrian::Update_Current_Position(pos2d new_pos){
    current_position = new_pos;    
};

}
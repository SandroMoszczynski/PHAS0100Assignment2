#include "sfmPedestrian.h"
#include "sfmForces.h"
#include <cmath>


namespace sfm{

dir2d &Forces::desired_position(){
    pos2d current_pos  = Return_Current_Position();
    pos2d origin = Return_Origin();
    dir2d length = origin  - current_pos;
    dir2d desired_direction(length[1] / length.length(), length[0]/ length.length());
    return desired_direction;
};

vec2d &Forces::attractive_force(){
    vec2d force;
    force = (desired_position()*Return_Speed() - Return_Velocity())*(1/Return_Rest_time());
    return force;

};

//sfm::dir2d &repulsive_force(){};
//sfm::dir2d &border_repulsive(){};

}
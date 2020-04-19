#include "sfmForces.h"
#include <cmath>
#include <iostream>


namespace sfm{


dir2d &Forces::desired_direction(dir2d &final_position){
    pos2d current_pos  = Return_Current_Position();
    pos2d origin = Return_Origin();
    dir2d length = origin  - current_pos;
    dir2d finalpos(length[1] / length.length(), length[0]/ length.length());
    final_position = finalpos;
    return final_position;
};

vec2d &Forces::attractive_force(vec2d &force){
    vec2d force_temp;
    dir2d final_position;
    force_temp = (desired_direction(final_position)*Return_Speed() - Return_Velocity())*(1/Return_Rest_time());
    force = force_temp;
    return force;
};

double &Forces::elipse(std::shared_ptr<Forces>pedesa, std::shared_ptr<Forces>pedesb, double &elipse, double step_width){
    double b;
    dir2d diff_pedestrians = pedesa->Return_Current_Position() - pedesb->Return_Current_Position();
    dir2d posb = pedesb->desired_direction(posb);
    b = sqrt(pow(diff_pedestrians.length() + (diff_pedestrians-posb*step_width).length(),2)- pow(step_width,2));
    elipse = b/2;
    return elipse;


};

//sfm::dir2d &repulsive_force(){};
//sfm::dir2d &border_repulsive(){};

}
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
    return final_position; // .length() of this seems to always be 1
};

dir2d &Forces::attractive_force(dir2d &force){
    dir2d force_temp;
    dir2d final_position;
    force_temp = (desired_direction(final_position)*Return_Speed() - Return_Velocity())*(1/Return_Rest_time());
    force = force_temp;
    return force;
};

double &Forces::elipse(std::shared_ptr<Forces>pedesa, std::shared_ptr<Forces>pedesb, double &elipse){
    double b;
    dir2d diff_pedestrians = pedesa->Return_Current_Position() - pedesb->Return_Current_Position();
    dir2d posb = pedesb->desired_direction(posb);
    double step_width = pedesb->Return_Speed()*2;
    b = sqrt(pow(diff_pedestrians.length() + (diff_pedestrians-posb*step_width).length(),2)- pow(step_width,2));
    elipse = b/2;
    return elipse;
};

dir2d &Forces::repulsive_force(std::shared_ptr<Forces>pedesa, std::shared_ptr<Forces>pedesb, double &elipse, dir2d &Forces){
    dir2d Force;
    double b;
    dir2d unit_length = pedesa->Return_Current_Position()- pedesb->Return_Current_Position();;
    dir2d unit_direction(unit_length[1] / unit_length.length(), unit_length[0]/ unit_length.length());
    b = pedesa->elipse(pedesa,pedesb,b);
    double Vab = 2.1*exp(-b/0.3);
    Force = unit_direction*(-(1/0.3)*Vab);
    Forces = Force;
    return Forces;
};

// double &Forces::fov(double f, double &fov){
//     double w;
//     w = f.length() cos(phi)
//     fov = w;
//     return fov;
// };

//sfm::dir2d &border_repulsive(){};

}
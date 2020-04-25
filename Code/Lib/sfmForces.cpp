#include "sfmForces.h"
#include <cmath>
#include <iostream>

namespace sfm{

//this class calculates what the vector for the pedestrians movement should be
//i.e where they want to go, the out put .length() should always be 1. The if loops are;
//  1. this one is  used for creating directional pedestrians. If the destination is set to
//{0,0}, which should be impossible under normal circumstances, then the length vector is set to
//the desired direction. this only changes when bumping into borders or other pedestrians.
//  2. this if loop stop errors being thrown out should a pedestrian every reach their destination.
dir2d &Forces::desired_direction(dir2d &final_position){
    pos2d current_pos  = Return_Current_Position();
    pos2d destination = Return_Destination();
    dir2d length;
    if(destination[0] == 0 && destination[1] == 0){ 
        dir2d velocity = Return_Velocity();
        length = velocity;
    }
    else{
        length = destination  - current_pos;
    }
    if(length[0] == 0 && length[1] == 0){
        std::cout << "Destination Reached" << std::endl;
        final_position= {current_pos[1],current_pos[0]};
    }
    else{
        dir2d finalpos(length[1] / length.length(), length[0]/ length.length());
        final_position = finalpos;
    }
    return final_position;
};

//this calculates how much of a force should be added to where the pedestrian wants to go,
//based on the vector direction given in desired_direction() and the pedestrians velocity
//and max speed
dir2d &Forces::attractive_force(dir2d &force){
    dir2d force_temp;
    dir2d final_position;
    force_temp = (desired_direction(final_position)*Return_Speed() - Return_Velocity())*(1/Return_Rest_time());
    force = force_temp;
    return force;
};

//this function calculates a little eliptical bubble around each pedestrian
//this is based on how close each other pedestrian is to them, the closer they are,
//the smaller the semi major axis will be, this in turns makes the repulsive force greater
double &Forces::elipse(std::shared_ptr<Forces>pedesb, double &elipse, double delta_t){
    double b;
    dir2d diff_pedestrians = Return_Current_Position() - pedesb->Return_Current_Position();
    dir2d posb = pedesb->desired_direction(posb);
    double step_width = pedesb->Return_Speed()*delta_t;
    b = sqrt(pow(diff_pedestrians.length() + (diff_pedestrians-(posb*step_width)).length(),2)- pow(step_width,2));
    elipse = b/2;
    return elipse;
};

//this calculates the pedestrian-pedestrian repulsive force. note that sigma is increased so that
//there is a larger force in a greater area. with 0.3 default i found pedestrians didnt mind
//getting a bit too close. especially with the current enviroment i felt this was unrealistic.
// The if loop here covers if the same pedestrian is asked to figure out the repulsive force against him
// this stops nan values for the force. The unit vector is nessecery to know which direction the
//force should be directed.
dir2d &Forces::repulsive_force(std::shared_ptr<Forces>pedesb, dir2d &Forces, double delta_t){
    dir2d Force;
    double b;
    double V_zero = 2.1;
    double sigma = 0.8;
    dir2d unit_length = Return_Current_Position()- pedesb->Return_Current_Position();
    if(unit_length[0] == 0 && unit_length[1] == 0){
        Forces = {0,0};
    }
    else{
    dir2d unit_direction(unit_length[1] / unit_length.length(), unit_length[0]/ unit_length.length());
    b = elipse(pedesb,b, delta_t);
    double Vab = V_zero*exp(-b/sigma);
    Force = unit_direction*((1/sigma)*Vab);
    Forces = Force;
    }
    return Forces;
};

//this sets the value of the repulsive forces to 0.5 or 1 depending on if the two pedestrians
//can see each other.
double &Forces::fov(dir2d rep_force,dir2d des_dir, double &fov,double phi, double c){
    double w;
    double ff;
    ff = rep_force*des_dir;
    w = rep_force.length()*cos(phi/2); 
    if (ff >= w){
        fov = 1;
    }
    else{
        fov = c;
    }
    return fov;
};

//This method calculates the border repulsive force should a pesdestrian stray too close to it. I chose to calculate the
//vector based on 10 points along the top and bottom of the y axis. this means that the pedestrians are free to overlap at 
//the ends of the corridor. This also means that the repulsive force is 10 times greater than it should be. I corrected this
//but found that it was very easy to get an interger overflow for the speed being too great for the repulsive force to overcome
//it. Thus i left the repulsive force higher than it should be for realism. This is also why R is almost 1m.
//The calculated repulsive force should cancel out midway, which is why the top and bottom are summed together. 
dir2d &Forces::border_repulsive(dir2d &Forces){
    std::vector<std::pair<double,double>> top_vec = {{0,10},{5,10},{10,10},{15,10},{20,10},{25,10},{30,10},{35,10},{40,10},{45,10},{50,10}};
    std::vector<std::pair<double,double>> bottom_vec = {{0,0},{5,0},{10,0},{15,0},{20,0},{25,0},{30,0},{35,0},{40,0},{45,0},{50,0}};
    double U_zero = 10;
    double R = 0.9;
    dir2d total_top;
    dir2d total_bot;
    dir2d total_temp;
    pos2d current_pos  = Return_Current_Position();
    for(int i = 0;i < top_vec.size(); ++i){
        vec2d top(top_vec[i].first,top_vec[i].second);
        vec2d bot(bottom_vec[i].first,bottom_vec[i].second);
        dir2d dis_to_top(current_pos[1]-top[1],current_pos[0]-top[0]);
        dir2d dis_to_bot(current_pos[1]-bot[1],current_pos[0]-bot[0]);
        dir2d unit_to_top(dis_to_top[1] / dis_to_top.length(), dis_to_top[0]/ dis_to_top.length());
        dir2d unit_to_bot(dis_to_bot[1] / dis_to_bot.length(), dis_to_bot[0]/ dis_to_bot.length());
        dir2d force_top = U_zero*exp(-dis_to_top.length()/R)*(1/R)*unit_to_top;
        dir2d force_bot = U_zero*exp(-dis_to_bot.length()/R)*(1/R)*unit_to_bot;
        total_top = total_top + force_top;
        total_bot = total_bot + force_bot;
    }
    total_temp = total_top + total_bot; 
    //Forces = {total_temp[1]/top_vec.size(),total_temp[0]/top_vec.size()};
    Forces = {total_temp[1],total_temp[0]};
    return Forces;
};

//this takes each pedestrian and calculates the resultant forces against every over pedestrian and the barriers. due to 
//{0,0} values in certain places the pedestrian can be safely resolved against itself. there is a value for delta_t incase 
//it is ever changed in the visualiser or open_mp function.
dir2d &Forces::Resultant_force(std::vector<std::shared_ptr<sfm::Forces> >Pedestrians, dir2d &Forces, double delta_t){
    dir2d rep_force;   
    for(int i = 0;i < Pedestrians.size();++i){
        dir2d temp_force = repulsive_force(Pedestrians[i],rep_force, delta_t);
        rep_force = rep_force + temp_force;
    }
    dir2d bor_repul = border_repulsive(bor_repul);
    dir2d ata_force = attractive_force(ata_force);
    dir2d des_dir = desired_direction(des_dir);
    double Fov = fov(rep_force,des_dir,Fov);
    Forces = (rep_force)*Fov + bor_repul + ata_force;
    return Forces;
};

}
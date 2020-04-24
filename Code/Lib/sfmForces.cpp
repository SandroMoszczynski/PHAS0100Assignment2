#include "sfmForces.h"
#include <cmath>
#include <iostream>

namespace sfm{

dir2d &Forces::desired_direction(dir2d &final_position){
    pos2d current_pos  = Return_Current_Position();
    pos2d destination = Return_Destination();
    if(destination[0] == 0 && destination[1] == 0){ //this makes the target directional, destination should never be 0,0
        dir2d velocity = Return_Velocity();
        // std::cout << "is this working?" << std::endl;
        destination = {current_pos[1]+velocity[1],current_pos[0]+velocity[0]};
    }
    dir2d length = destination  - current_pos;
    if(length[0] == 0 && length[1] == 0){
        std::cout << "Destination Reached" << std::endl; // should make this message be more individual later
        final_position= {current_pos[1],current_pos[0]};
    }
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

double &Forces::elipse(std::shared_ptr<Forces>pedesb, double &elipse, double delta_t){
    double b;
    dir2d diff_pedestrians = Return_Current_Position() - pedesb->Return_Current_Position();
    dir2d posb = pedesb->desired_direction(posb);
    double step_width = pedesb->Return_Speed()*delta_t;
    b = sqrt(pow(diff_pedestrians.length() + (diff_pedestrians-(posb*step_width)).length(),2)- pow(step_width,2));
    elipse = b/2;
    return elipse;
};

dir2d &Forces::repulsive_force(std::shared_ptr<Forces>pedesb, dir2d &Forces, double delta_t){
    dir2d Force;
    double b;
    double V_zero = 2.1;
    double sigma = 0.3;
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

double &Forces::fov(dir2d rep_force,dir2d des_dir, double &fov,double phi, double c){
    double w;
    double ff;
    ff = rep_force*des_dir;
    w = rep_force.length()*cos(phi/2); 
    //ff = sqrt(pow(rep_force*des_dir,2));
    //w = sqrt(pow(rep_force.length()*cos(phi/2),2)); 
    if (ff >= w){
        fov = 1;
    }
    else{
        fov = c;
    }
    return fov;
};

dir2d &Forces::border_repulsive(dir2d &Forces){
    std::vector<std::pair<double,double>> top_vec = {{0,10},{5,10},{10,10},{15,10},{20,10},{25,10},{30,10},{35,10},{40,10},{45,10},{50,10}};
    std::vector<std::pair<double,double>> bottom_vec = {{0,0},{5,0},{10,0},{15,0},{20,0},{25,0},{30,0},{35,0},{40,0},{45,0},{50,0}};
    double U_zero = 10; //m^2/s^-2
    double R = 0.2; //m
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
    total_temp = total_top + total_bot; // chose this as it would cancel out in the middle, which is ideal
    Forces = {total_temp[1]/top_vec.size(),total_temp[0]/top_vec.size()};
    //Forces = {total_temp[1],total_temp[0]};
    return Forces;
};

dir2d &Forces::Resultant_force(std::vector<std::shared_ptr<sfm::Forces> >Pedestrians, dir2d &Forces, double delta_t){
    dir2d rep_force;   
    for(int i = 0;i < Pedestrians.size();++i){
        dir2d temp_force = repulsive_force(Pedestrians[i],rep_force, delta_t);
        rep_force = rep_force + temp_force;
    }
    //std::cout << rep_force[1] << " ," << rep_force[0] << "ped rep force" << std::endl;
    dir2d bor_repul = border_repulsive(bor_repul);
    //std::cout << bor_repul[1] << " ," << bor_repul[0] << " border repulsive" << std::endl;
    dir2d ata_force = attractive_force(ata_force);
    //std::cout << ata_force[1] << " ," << ata_force[0] << " destination attractive" << std::endl;
    dir2d des_dir = desired_direction(des_dir);
    //std::cout << des_dir[1] << " ," << des_dir[0] << " desired direction" << std::endl;
    double Fov = fov(rep_force,des_dir,Fov);
    //std::cout << Fov << " fov, 1 or 0.5" << std::endl;
    Forces = (rep_force)*Fov + bor_repul+ ata_force;
    //std::cout << Forces[1] << " ," << Forces[0] << " resultant force" << std::endl;
    return Forces;
};

}
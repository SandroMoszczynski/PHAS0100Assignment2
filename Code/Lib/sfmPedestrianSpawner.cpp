#include "sfmPedestrianSpawner.h"
#include <random>
#include <iostream>

namespace sfm{


dir2d &Factory::R_Dir2d(dir2d &dir2d_out, double x_min, double x_max, double y_min, double y_max){
    std::default_random_engine generator;
    std::uniform_real_distribution<double> dir2d_gen_x(x_min,x_max); 
    std::uniform_real_distribution<double> dir2d_gen_y(y_min,y_max);
    dir2d dir2d_in = {dir2d_gen_x(generator), dir2d_gen_y(generator)};
    dir2d_out = dir2d_in;
    return dir2d_out;
};


pos2d &Factory::R_Pos2d(pos2d &pos2d_out,double x_min, double x_max, double y_min, double y_max){
    std::default_random_engine generator;
    std::uniform_real_distribution<double> pos2d_gen_x(x_min,x_max);  
    std::uniform_real_distribution<double> pos2d_gen_y(y_min,y_max);  
    pos2d pos2d_in = {pos2d_gen_x(generator), pos2d_gen_y(generator)};
    pos2d_in = pos2d_out;
    return pos2d_out;
};

double &Factory::R_Doub(double &double_out, double double_min, double double_max){
    std::default_random_engine generator;
    std::uniform_real_distribution<double> double_gen(double_min,double_max); 
    double double_in  = double_gen(generator);
    double_out = double_in;
    return double_out;
};

std::vector<std::shared_ptr<sfm::Forces>> &Factory::Spawner(
    std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
    int no_pedestrians,
    double vel_min_x,
    double vel_max_x,
    double vel_min_y,
    double vel_max_y,
    double speed_min,
    double speed_max,
    double x_min_start, 
    double x_max_start, 
    double y_min_start, 
    double y_max_start,
    double x_min_dest, 
    double x_max_dest, 
    double y_min_dest, 
    double y_max_dest,
    double rest_min,
    double rest_max){

    for(int i=0;i<no_pedestrians;++i){ 
        sfm::pos2d origin = R_Pos2d(origin,x_min_start,x_max_start,y_min_start,y_max_start);
        sfm::pos2d destination = R_Pos2d(origin,x_min_dest,x_max_dest,y_min_dest,y_max_dest);
        sfm::dir2d velocity = R_Dir2d(velocity, vel_min_x,vel_max_x,vel_min_x,vel_max_x);
        sfm::pos2d current_position = origin;
        double speed = R_Doub(speed, speed_min, speed_max);
        double rest_time = R_Doub(rest_time, rest_min,rest_max);
        sfm::Forces ped_spawn(origin,destination,velocity,current_position,speed,rest_time);
        std::shared_ptr<sfm::Forces> pointer(new sfm::Forces(ped_spawn));
        pedestrians.emplace_back(pointer);
    }                                                        
    return pedestrians;
};

std::vector<std::shared_ptr<sfm::Forces>> &Factory::Targeted(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        int no_pedestrians,
        pos2d Destination,
        pos2d min_values,
        pos2d max_values,
        pos2d speed,
        pos2d rest){
    
    double destination_x = Destination[1];
    double destination_y = Destination[0]; 
    double speed_min = speed[1];
    double speed_max = speed[0];
    double rest_min = rest[1];
    double rest_max = rest[0];
    double x_min = min_values[1];
    double y_min = min_values[0];
    double x_max = max_values[1];
    double y_max = max_values[0];
    double vel_min_x = 0.1;
    double vel_max_x = 1.5;
    double vel_min_y = 0.1;
    double vel_max_y = 1.5;
    
    pedestrians = Spawner(pedestrians, no_pedestrians,
        vel_min_x, vel_max_x, vel_min_y, vel_max_y,
        speed_min, speed_max, 
        x_min, x_max, y_min, y_max,
        destination_x, destination_x, destination_y, destination_y, 
        rest_min, rest_max);
    return pedestrians;
    };
    
std::vector<std::shared_ptr<sfm::Forces>> &Factory::Directional(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        int no_pedestrians, 
        dir2d Direction,
        pos2d min_values,
        pos2d max_values,
        pos2d speed,
        pos2d rest){

    double destination_x = 0;
    double destination_y = 0;
    double vel_x = Direction[1];
    double vel_y = Direction[0];
    double x_min = min_values[1];
    double y_min = min_values[0];
    double x_max = max_values[1];
    double y_max = max_values[0];
    double speed_min = speed[1];
    double speed_max = speed[0];
    double rest_min = rest[1];
    double rest_max = rest[0];
    
    pedestrians = Spawner(pedestrians, no_pedestrians,
        vel_x, vel_x, vel_y, vel_y,
        speed_min, speed_max,
        x_min, x_max, y_min, y_max,
        destination_x, destination_x, destination_y, destination_y,
        rest_min, rest_max);
    return pedestrians;
    };

std::vector<std::shared_ptr<sfm::Forces>> &Factory::Uniform(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        std::string Type_pedestrian,
        int no_pedestrians,
        pos2d destination){

    if(Type_pedestrian == "Targeted","Tar","T","t"){
        pedestrians = Targeted(pedestrians,no_pedestrians,destination);
    } 
    else if(Type_pedestrian == "Directional","Dir","D","d"){
        dir2d direction = {destination[1],destination[0]};
        pedestrians = Directional(pedestrians,no_pedestrians,direction);
    }
    else{
        std::cout << "Valid types are; Targeted,Tar,T,t or Directional,Dir,D,d." << std::endl;
    }
    return pedestrians;
    };    

std::vector<std::shared_ptr<sfm::Forces>> &Factory::Distributed(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        std::string Type_pedestrian,
        int no_pedestrians,
        pos2d destination,
        pos2d min_values,
        pos2d max_values){

    if(Type_pedestrian == "Targeted","Tar","T","t"){
        pedestrians = Targeted(pedestrians,no_pedestrians,destination, min_values, max_values);
    } 
    else if(Type_pedestrian == "Directional","Dir","D","d"){
        dir2d direction = {destination[1],destination[0]};
        pedestrians = Directional(pedestrians,no_pedestrians,direction, min_values, max_values);
    }
    else{
        std::cout << "Valid types are; Targeted,Tar,T,t or Directional,Dir,D,d." << std::endl;
    }
    return pedestrians;
    };  

}



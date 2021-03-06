#include "sfmPedestrianSpawner.h"
#include <random>
#include <iostream>
#include <chrono>

namespace sfm{

//this creates a dir2d vector with random values between a specified range.
//each of the functions uses the systemclock to create a random seed,
//this means its almost impossible for the values to all be the same
dir2d &Factory::R_Dir2d(dir2d &dir2d_out, double x_min, double x_max, double y_min, double y_max){
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> dir2d_gen_x(x_min,x_max); 
    std::uniform_real_distribution<double> dir2d_gen_y(y_min,y_max);
    dir2d dir2d_in = {dir2d_gen_x(generator), dir2d_gen_y(generator)};
    dir2d_out = dir2d_in;
    return dir2d_out;
};

//does the same as above but for a pos2d vector.
pos2d &Factory::R_Pos2d(pos2d &pos2d_out,double x_min, double x_max, double y_min, double y_max){
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> pos2d_gen_x(x_min,x_max);  
    std::uniform_real_distribution<double> pos2d_gen_y(y_min,y_max);  
    pos2d pos2d_in = {pos2d_gen_x(generator), pos2d_gen_y(generator)};
    pos2d_out = pos2d_in;
    return pos2d_out;
};

//as above but for a double.
double &Factory::R_Doub(double &double_out, double double_min, double double_max){
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> double_gen(double_min,double_max); 
    double double_in  = double_gen(generator);
    double_out = double_in;
    return double_out;
};

//this is the pedestrian spawner. it creates an instance of a class and then
//creates a pointer for it, it then inputs it into a vector of pointers. 
// each of the values except no_pedestrians has a default value to allow for
// very quick random generation. the current position is also set to origin,
// which should always be where it starts. 
std::vector<std::shared_ptr<sfm::Forces>> &Factory::Spawner(
    std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
    int no_pedestrians,
    double vel_min_x,
    double vel_max_x,
    double vel_min_y,
    double vel_max_y,
    double speed_min,
    double speed_max,
    double x_start_min, 
    double x_start_max, 
    double y_start_min, 
    double y_start_max,
    double x_dest_min, 
    double x_dest_max, 
    double y_dest_min, 
    double y_dest_max,
    double rest_min,
    double rest_max){

    for(int i=0;i<no_pedestrians;++i){ 
        sfm::pos2d origin = R_Pos2d(origin,x_start_min,x_start_max,y_start_min,y_start_max);  
        sfm::pos2d destination = R_Pos2d(destination,x_dest_min,x_dest_max,y_dest_min,y_dest_max);
        sfm::dir2d velocity = R_Dir2d(velocity, vel_min_x,vel_max_x,vel_min_y,vel_max_y);
        sfm::pos2d current_position = origin;
        double speed = R_Doub(speed, speed_min, speed_max);
        double rest_time = R_Doub(rest_time, rest_min,rest_max);
        sfm::Forces ped_spawn(origin,destination,velocity,current_position,speed,rest_time);
        std::shared_ptr<sfm::Forces> pointer(new sfm::Forces(ped_spawn));
        pedestrians.emplace_back(pointer);
    }                                                    
    return pedestrians;
};

//this is the first time saving function. it needs less inputs but has a few defaults
//incase you need to change the values. in this case the factory needs a destination,
//and a number of pedestrians. but can have specified speeds and rest times. 
//They start at rest.
std::vector<std::shared_ptr<sfm::Forces>> &Factory::Targeted(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        int no_pedestrians,
        dir2d Destination_x,
        dir2d Destination_y,
        dir2d x_values,
        dir2d y_values,
        dir2d speed,
        dir2d rest){
    
    double destination_x_min = Destination_x[1];
    double destination_x_max = Destination_x[0];
    double destination_y_min = Destination_y[1]; 
    double destination_y_max = Destination_y[0];
    double speed_min = speed[1];
    double speed_max = speed[0];
    double rest_min = rest[1];
    double rest_max = rest[0];
    double x_min = x_values[1];
    double x_max = x_values[0];
    double y_min = y_values[1];    
    double y_max = y_values[0];
    double vel_min_x = 0;
    double vel_max_x = 0;
    double vel_min_y = 0;
    double vel_max_y = 0;
    
    pedestrians = Spawner(pedestrians, no_pedestrians,
        vel_min_x, vel_max_x, vel_min_y, vel_max_y,
        speed_min, speed_max, 
        x_min, x_max, y_min, y_max,
        destination_x_min, destination_x_max, destination_y_min, destination_y_max, 
        rest_min, rest_max);
    return pedestrians;
    };
    
//this factory creates directional pedestrians. In order to do this the desired_direction()
//method in forces.cpp uses an if statement for when the destination is set to {0,0} that will
//give it the behaviour nessecery. The required inputs for this class are the number of pedestrians
//and the direction requested. Optionally you can change the start location and the speed
std::vector<std::shared_ptr<sfm::Forces>> &Factory::Directional(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        int no_pedestrians, 
        dir2d Direction,
        dir2d x_values,
        dir2d y_values,
        dir2d speed,
        dir2d rest){

    double desti_x = 0;
    double desti_y = 0;
    double vel_x = Direction[1];
    double vel_y = Direction[0];
    double x_min = x_values[1];
    double x_max = x_values[0];
    double y_min = y_values[1];    
    double y_max = y_values[0];
    double speed_min = speed[1];
    double speed_max = speed[0];
    double rest_min = rest[1];
    double rest_max = rest[0];
  
    pedestrians = Spawner(pedestrians, no_pedestrians,
        vel_x, vel_x, vel_y, vel_y,
        speed_min, speed_max,
        x_min, x_max, y_min, y_max,
        desti_x, desti_x, desti_y, desti_y,
        rest_min, rest_max);
    return pedestrians;
    };

//this is the shortest factory type, with the expection of default spawner. this method takes
// the type of pedestrian,the the number of pedestrians and the destination of the pedestrians as
//input. If directional pedestrian type is chosen, the second destination isnt used. 
std::vector<std::shared_ptr<sfm::Forces>> &Factory::Uniform(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        std::string Type_pedestrian,
        int no_pedestrians,
        dir2d Destination_x, 
        dir2d Destination_y){

    if(Type_pedestrian == "Targeted"){
        pedestrians = Targeted(pedestrians,no_pedestrians,Destination_x,Destination_y);
    } 
    else if(Type_pedestrian == "Directional"){
            pedestrians = Directional(pedestrians,no_pedestrians,Destination_x);
    }
    else{
        std::cout   << "Incorrect use of Uniform Type_pedestrian, Valid types are; \n"
                    << "Targeted (or subset of letters) \n"
                    << "Directional(or a subset of letters)" << std::endl;
    }
    return pedestrians;
    };    

// the last method takes the above required inputs, plus inputs of where the pedestrians
//should start. note this also has a dummy variable if directional is chosen.
std::vector<std::shared_ptr<sfm::Forces>> &Factory::Distributed(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        std::string Type_pedestrian,
        int no_pedestrians,
        dir2d Destination_x, 
        dir2d Destination_y,
        dir2d x_values,
        dir2d y_values){

    if(Type_pedestrian == "Targeted"){
        pedestrians = Targeted(pedestrians,no_pedestrians,Destination_x,Destination_y, x_values, y_values);
    } 
    else if(Type_pedestrian == "Directional"){
        pedestrians = Directional(pedestrians,no_pedestrians,Destination_x, x_values, y_values);
    }
    else{
        std::cout   << "Incorrect use of Distributed Type_pedestrian, Valid types are; \n"
                    << "Targeted (or subset of letters) \n"
                    << "Directional(or a subset of letters)" << std::endl;
    }
    return pedestrians;
    };  

}



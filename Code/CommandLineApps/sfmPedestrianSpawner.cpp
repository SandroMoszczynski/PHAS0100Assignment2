#include <iostream>
#include <vector>
#include "sfmMyFunctions.h"
#include "sfmExceptionMacro.h"
#include "sfmBasicTypes.h"
#include "sfmForces.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <random>

int main()
{
    int no_pedestrians = 20;
    //std::cin >> no_pedestrians;
    std::default_random_engine generator;
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    for(int i=0;i<no_pedestrians;++i){
        std::uniform_real_distribution<double> origin_gen_x(0,50);  
        std::uniform_real_distribution<double> origin_gen_y(0,10);   
        std::uniform_real_distribution<double> destination_gen_x(0,50);  
        std::uniform_real_distribution<double> destination_gen_y(0,10);  
        std::uniform_real_distribution<double> velicty_gen(0,1);
        std::uniform_real_distribution<double> current_position_gen(0,10);  
        std::uniform_real_distribution<double> speed_gen(0.1,34);   
        std::uniform_real_distribution<double> resttime_gen(0.1,2);  
        sfm::pos2d origin = {origin_gen_x(generator),origin_gen_y(generator)};
        sfm::pos2d destination = {destination_gen_x(generator), destination_gen_y(generator)};
        sfm::dir2d velocity = {velicty_gen(generator), velicty_gen(generator)};
        sfm::pos2d current_position = {current_position_gen(generator), current_position_gen(generator)};
        double speed = speed_gen(generator);
        double rest_time = resttime_gen(generator);
        sfm::Forces ped_spawn(origin,destination,velocity,current_position,speed,rest_time);
        std::shared_ptr<sfm::Forces> pointer(new sfm::Forces(ped_spawn));
        pedestrians.emplace_back(pointer);
    }

    
    
    


    //another for loop over all time

    double dt = 0.1; //s
    double finish_time_s = 1;//second
    //std::cin >> dt;
    //std::cin >> finish_time_s;
    for(int t=0; t<(finish_time_s/dt);++t){
        
        //next a for loop that goes through each pedestrian and calculates its resultant force and then updates the new qualities
        std::vector<sfm::dir2d>Totalforce;
        for(int j=0; j<pedestrians.size();++j){
            sfm::dir2d temp_force;
            Totalforce.push_back(pedestrians[j]->Resultant_force(pedestrians,temp_force, dt));
            sfm::dir2d  new_velocity = (Totalforce[j] + pedestrians[j]->Return_Velocity())*dt;
            if(new_velocity.length() > 1.3*pedestrians[j]->Return_Speed()){
                new_velocity = new_velocity*(1.3*pedestrians[j]->Return_Speed()/new_velocity.length());
                }
                sfm::dir2d position(pedestrians[j]->Return_Current_Position()[1],pedestrians[j]->Return_Current_Position()[0]);
            sfm::pos2d new_position = {new_velocity[1]+position[1]*dt,new_velocity[0]+position[0]*dt};
            pedestrians[j]->Update_Velocity(new_velocity);
            pedestrians[j]->Update_Current_Position(new_position);
        }
        

    }



    return 0;
}

// std::cout << pedestrians[j]->Return_Velocity()[0] << "," << pedestrians[j]->Return_Velocity()[0] << " velocity after:" << j << std::endl;
// std::cout << pedestrians[j]->Return_Current_Position()[0] << "," << pedestrians[j]->Return_Current_Position()[0] << " position after:" << j << std::endl;
//std::cout << Totalforce[j][0] << "," << Totalforce[j][1] << std::endl;
//std::cout << "error here" << std::endl;
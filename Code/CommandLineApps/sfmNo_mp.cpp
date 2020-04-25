#include <iostream>
#include <vector>
#include "sfmMyFunctions.h"
#include "sfmExceptionMacro.h"
#include "sfmBasicTypes.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <random>
#include "sfmPedestrianSpawner.h"
#include <iomanip>
#include <chrono>
#include <ctime>

std::vector<std::shared_ptr<sfm::Forces> > &update_pedestrian(std::vector<std::shared_ptr<sfm::Forces> > &pedestrians)
{
    double dt = 0.1; //s
    double finish_time_s = 10;//second
    double v_max = 1.3;
    sfm::dir2d temp_force;
    sfm::dir2d new_velocity;
    sfm::dir2d position;
    sfm::pos2d new_position;
    for(int t=0; t<(finish_time_s/dt);++t){
        for(int j=0; j<pedestrians.size();++j){               
            temp_force = pedestrians[j]->Resultant_force(pedestrians,temp_force, dt);
            new_velocity = (temp_force*dt) + pedestrians[j]->Return_Velocity();
            if(new_velocity.length() > v_max*pedestrians[j]->Return_Speed()){
                new_velocity = new_velocity*(v_max*pedestrians[j]->Return_Speed()/new_velocity.length());
                }
            position = {pedestrians[j]->Return_Current_Position()[1],pedestrians[j]->Return_Current_Position()[0]};
            new_position = {position[1]+(new_velocity[1]*dt),(position[0]+new_velocity[0]*dt)};
            pedestrians[j]->Update_Velocity(new_velocity);
            pedestrians[j]->Update_Current_Position(new_position);
        }
    }
    return pedestrians;
}

std::vector<std::shared_ptr<sfm::Forces> > &Create_Pedestrian(std::vector<std::shared_ptr<sfm::Forces> > &pedestrians)
{
    int no_pedestrians = 500; 
    sfm::dir2d left_side_x(1,1);
    sfm::dir2d left_side_y(0.1,9.9);
    sfm::dir2d dest_left_x(48,49);
    sfm::dir2d dest_left_y(0.1,9.9);
    sfm::dir2d right_side_x(48,49);
    sfm::dir2d right_side_y(0.1,9.9);
    sfm::dir2d dest_right_x(1,2);
    sfm::dir2d dest_right_y(0.1,9.9);
    sfm::dir2d direc1(1,0);
    sfm::dir2d direc2(-1,0);

    std::vector<std::shared_ptr<sfm::Forces> >pedestrians1;
    pedestrians1 = sfm::Factory::Directional(pedestrians1,no_pedestrians/2,direc1,left_side_x,left_side_y);
    for(int point = 0; point < pedestrians1.size();++point){
        pedestrians.emplace_back(pedestrians1[point]);
    } 
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians2;
    pedestrians2 = sfm::Factory::Directional(pedestrians2,no_pedestrians/2,direc2,right_side_x,right_side_y);
    for(int point = 0; point < pedestrians2.size();++point){
        pedestrians.emplace_back(pedestrians2[point]);
    }
    return pedestrians;   
}

int main()
{
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    std::cout << "start" << std::endl;
    std::clock_t c_start = std::clock();
    auto t_start = std::chrono::high_resolution_clock::now();
    pedestrians = Create_Pedestrian(pedestrians);
    pedestrians = update_pedestrian(pedestrians);
    std::clock_t c_end = std::clock();
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << "stop" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "CPU time used: "
              << 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC << " ms\n"
              << "Wall clock time passed: "
              << std::chrono::duration<double, std::milli>(t_end-t_start).count()
              << " ms\n";
    return 0;
}

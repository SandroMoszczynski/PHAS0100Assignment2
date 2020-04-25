#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <array>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

#include "sfmMyFunctions.h"
#include "sfmExceptionMacro.h"
#include "sfmBasicTypes.h"
#include "sfmPedestrianSpawner.h"

#include "sfmVisualiser.h"

int main(int argc, char** argv)
{
    // Needed these to define viewer world	
    double world_width_x = 50.0;
    double world_height_y = 10.0;
    int no_pedestrians = 100;
    //std::cin >> no_pedestrians;

    // Create viewer and initialise with required number of pedestrians
    sfm::Visualiser viewer(no_pedestrians, world_width_x, world_height_y);

    // Or create with non-default window size, set to 20 pixels/meter
    //  sfm::Visualiser viewer(n_pedestrians, world_width_x, world_height_y, 20.0);

    // Example if you want to change the base marker width (in pixels) 
    //viewer.SetMarkerSize(1.0);

    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    //pedestrians = sfm::Factory::Spawner(pedestrians, no_pedestrians);
    // pedestrians = sfm::Factory::Spawner(pedestrians, no_pedestrians,
    // 1.5, 1.5, 0, 0,
    // 1, 1.5,
    // 0.1, 49.9, 0.1, 9.9,
    // 0, 0, 0, 0,
    // 0.1, 0.1);

    // pedestrians = sfm::Factory::Spawner(pedestrians, no_pedestrians,
    // 1.5, 1.5, 0, 0,
    // 1, 1.5,
    // 0.1, 49.9, 0.1, 9.9,
    // 0, 0, 0, 0,
    // 0.1, 0.1);

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
    // pedestrians = sfm::Factory::Directional(pedestrians,no_pedestrians,{2,0},{1,1},{5,5},{1,1},{0.1,0.1}); 
    // pedestrians = sfm::Factory::Distributed(pedestrians,"Directional",no_pedestrians,direc,direc,left_side_x,left_side_y);
    // std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    // pedestrians = sfm::Factory::Distributed(pedestrians,"Targeted",no_pedestrians,dest_final,left_side_min,left_side_max);      

    
    // std::cout << "no error" << std::endl;

    //another for loop over all time
    double dt = 0.1; //s
    double finish_time_s = 100;//second
    double v_max = 1.3;
    //std::cin >> v_max;
    //std::cin >> dt;
    //std::cin >> finish_time_s;
    for(int t=0; t<(finish_time_s/dt);++t){

        //next a for loop that goes through each pedestrian and calculates its resultant force and then updates the new qualities
        for(int j=0; j<pedestrians.size();++j){
            viewer.SetPedestrian(j, pedestrians[j]->Return_Current_Position()[1],pedestrians[j]->Return_Current_Position()[0],pedestrians[j]->Return_Velocity()[1],pedestrians[j]->Return_Velocity()[0]);
            sfm::dir2d temp_force = pedestrians[j]->Resultant_force(pedestrians,temp_force, dt);
            sfm::dir2d  new_velocity = (temp_force*dt) + pedestrians[j]->Return_Velocity();
            //std::cout << new_velocity.length() << ", " << v_max*pedestrians[1]->Return_Speed() << std::endl;
            if(new_velocity.length() > v_max*pedestrians[j]->Return_Speed()){
                new_velocity = new_velocity*(v_max*pedestrians[j]->Return_Speed()/new_velocity.length());
                }
            sfm::dir2d position(pedestrians[j]->Return_Current_Position()[1],pedestrians[j]->Return_Current_Position()[0]);
            sfm::pos2d new_position = {position[1]+(new_velocity[1]*dt),(position[0]+new_velocity[0]*dt)};
            pedestrians[j]->Update_Velocity(new_velocity);
            pedestrians[j]->Update_Current_Position(new_position);
        }
        

        // Tell viewer to redraw scene
        viewer.UpdateScene();

        // Sleep for a bit so can see visualiser updating 
        std::this_thread::sleep_for (std::chrono::milliseconds(10));

    } 

    return 0;
}
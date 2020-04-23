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
#include "sfmForces.h"
#include "sfmPedestrianSpawner.h"

#include "sfmVisualiser.h"

int main(int argc, char** argv)
{
    // Needed these to define viewer world	
    double world_width_x = 50.0;
    double world_height_y = 10.0;
    int no_pedestrians = 20;
    //std::cin >> no_pedestrians;

    // Create viewer and initialise with required number of pedestrians
    sfm::Visualiser viewer(no_pedestrians, world_width_x, world_height_y);

    // Or create with non-default window size, set to 20 pixels/meter
    //  sfm::Visualiser viewer(n_pedestrians, world_width_x, world_height_y, 20.0);

    // Example if you want to change the base marker width (in pixels) 
    //viewer.SetMarkerSize(1.0);

    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians1 = 
        sfm::Factory::Uniform(pedestrians1,"t", 5, {0,5});
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians2 = 
        sfm::Factory::Directional(pedestrians2,5, {2,0});
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians3 = 
        sfm::Factory::Spawner(pedestrians3,20);    
    pedestrian.emplace_back(pedestrians1);
    pedestrian.emplace_back(pedestrians2);
    pedestrian.emplace_back(pedestrians3);


    //another for loop over all time
    double dt = 0.1; //s
    double finish_time_s = 50;//second
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
        std::this_thread::sleep_for (std::chrono::milliseconds(5));

    } 

    return 0;
}
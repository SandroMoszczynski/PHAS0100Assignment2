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

//this always seems to be highlighted but it complies fine
#include "sfmVisualiser.h"

int main(int argc, char** argv)
{
    //variables needed throughout the function
    double world_width_x = 50.0;
    double world_height_y = 10.0;
    int no_pedestrians = 100;
    int choice;
    double dt = 0.1;
    double finish_time_s = 100;
    double v_max = 1.3;
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;

    // Create viewer and initialise with required number of pedestrians
    //left here as its easier to reposition the viewer to where you would like it before
    //entering the selection, i left a second commented later incase someone didnt like
    //this, it is a little jarring at first.
    sfm::Visualiser viewer(no_pedestrians, world_width_x, world_height_y);

    // neatened up the interface to easily pick from three demos
    std::cout   << "Please Pick Demo of 100 pedestrians over 50 seconds; \n"
                << "1(Directional),2(Targeted) or 3(All random);) " << std::endl;
    std::cin >> choice;

    //this has all the pedestrians moving straight along the x axis to their opposite side
    if(choice == 1){
    
        //set up variables for directional functions
        sfm::dir2d left_side_x(1,2);
        sfm::dir2d left_side_y(0.1,9.9);
        sfm::dir2d right_side_x(48,49);
        sfm::dir2d right_side_y(0.1,9.9);
        sfm::dir2d direc1(1,0);
        sfm::dir2d direc2(-1,0);

        //creates 2 sets of pedestrians in the same format as in the open_mp file
        //essencially we create 2 sets of pedestrians and append them to the total.
        std::vector<std::shared_ptr<sfm::Forces> >pedestrians1;
        pedestrians1 = sfm::Factory::Distributed(pedestrians1,"Directional",no_pedestrians/2,direc1,direc1,left_side_x,left_side_y);
        for(int point = 0; point < pedestrians1.size();++point){
            pedestrians.emplace_back(pedestrians1[point]);
        } 
        std::vector<std::shared_ptr<sfm::Forces> >pedestrians2;
        pedestrians2 = sfm::Factory::Distributed(pedestrians2,"Directional",no_pedestrians/2,direc2,direc2,right_side_x,right_side_y);
        for(int point = 0; point < pedestrians2.size();++point){
            pedestrians.emplace_back(pedestrians2[point]);
        }   
    }

    //this has all the pedestrians moving straight along to a point on the opposite side
    else if(choice == 2){

        //variables for targeted pedestrians
        sfm::dir2d left_side_x(1,2);
        sfm::dir2d left_side_y(0.1,9.9);
        sfm::dir2d dest_left_x(48,49);
        sfm::dir2d dest_left_y(0.1,9.9);
        sfm::dir2d right_side_x(48,49);
        sfm::dir2d right_side_y(0.1,9.9);
        sfm::dir2d dest_right_x(1,2);
        sfm::dir2d dest_right_y(0.1,9.9);

        // this uses the distributed fixed class method
        std::vector<std::shared_ptr<sfm::Forces> >pedestrians1;
        pedestrians1 = sfm::Factory::Distributed(pedestrians1,"Targeted",no_pedestrians/2,dest_left_x,dest_left_y,left_side_x,left_side_y);
        for(int point = 0; point < pedestrians1.size();++point){
            pedestrians.emplace_back(pedestrians1[point]);
        } 
        std::vector<std::shared_ptr<sfm::Forces> >pedestrians2;
        pedestrians2 = sfm::Factory::Distributed(pedestrians2,"Targeted",no_pedestrians/2,dest_right_x,dest_right_y,right_side_x,right_side_y);
        for(int point = 0; point < pedestrians2.size();++point){
            pedestrians.emplace_back(pedestrians2[point]);
        } 

    }

    // this will randomly spawn 100 pedestrains at 100 randompoints
    else if(choice == 3){

        //very simple to spawn random pedestrians
        pedestrians = sfm::Factory::Spawner(pedestrians,no_pedestrians);
    }
    else{
        std::cout << "Pick one of the three demos" <<std::endl;
    }

    // Create viewer and initialise with required number of pedestrians
    //sfm::Visualiser viewer(no_pedestrians, world_width_x, world_height_y);

    //for loop over time
    for(int t=0; t<(finish_time_s/dt);++t){   

        //next a for loop that goes through each pedestrian and calculates its resultant force and then updates the new qualities
        //detailed explanation in open mp
        for(int j=0; j<pedestrians.size();++j){
            viewer.SetPedestrian(j, pedestrians[j]->Return_Current_Position()[1],
                                    pedestrians[j]->Return_Current_Position()[0],
                                    pedestrians[j]->Return_Velocity()[1],
                                    pedestrians[j]->Return_Velocity()[0]);
            sfm::dir2d temp_force = pedestrians[j]->Resultant_force(pedestrians,temp_force, dt);
            sfm::dir2d  new_velocity = (temp_force*dt) + pedestrians[j]->Return_Velocity();
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

        // Sleep for a bit so can see visualiser updating, i left it as 10 miliseconds at the 
        // actual sleep time of 100 ms seemed a bit too slow (0.1s dt)
        std::this_thread::sleep_for (std::chrono::milliseconds(100));

    } 

    return 0;
}
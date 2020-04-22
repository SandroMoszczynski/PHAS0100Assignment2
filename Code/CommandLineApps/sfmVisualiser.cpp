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

    // //pedestrian 1
    // sfm::pos2d origin1(1,5); 
    // sfm::pos2d destination1(49,5);
    // sfm::dir2d velocity1(1,0);
    // sfm::pos2d current_position1 = origin1;
    // double speed1 = 1.5;
    // double rest_time1 = 1;
    // //pedestrian 2
    // sfm::pos2d origin2(1,1);
    // sfm::pos2d destination2(44,9.5);
    // sfm::dir2d velocity2(1.5,0.5);
    // sfm::pos2d current_position2 = origin2;
    // double speed2 = 1.5;
    // double rest_time2 = 0.5;
    // //pedestrian 3
    // sfm::pos2d origin3(1,5); 
    // sfm::pos2d destination3(44,1);
    // sfm::dir2d velocity3(2,0.5);
    // sfm::pos2d current_position3 = origin3;
    // double speed3 = 1;
    // double rest_time3 = 0.5;
    // //creating classes through Forces
    // sfm::Forces test2_1(origin1, destination1,velocity1, current_position1, speed1, rest_time1);
    // sfm::Forces test2_2(origin2, destination2,velocity2, current_position2, speed2, rest_time2);
    // sfm::Forces test2_3(origin3, destination3,velocity3, current_position3, speed3, rest_time3);
    // //making a pointer vector of classes
    // std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    // std::shared_ptr<sfm::Forces> test_ptr1(new sfm::Forces(test2_1));
    // std::shared_ptr<sfm::Forces> test_ptr2(new sfm::Forces(test2_2));
    // std::shared_ptr<sfm::Forces> test_ptr3(new sfm::Forces(test2_3));
    // pedestrians.emplace_back(test_ptr1);
    // pedestrians.emplace_back(test_ptr2);
    // pedestrians.emplace_back(test_ptr3);



    std::default_random_engine generator;
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    for(int i=0;i<no_pedestrians;++i){
        std::uniform_real_distribution<double> origin_gen_x(0.1,49.9);  
        std::uniform_real_distribution<double> origin_gen_y(0.1,9.9);   
        std::uniform_real_distribution<double> destination_gen_x(0.1,49.9);  
        std::uniform_real_distribution<double> destination_gen_y(0.1,9.9);  
        std::uniform_real_distribution<double> velocity_gen(0,1); 
        std::uniform_real_distribution<double> speed_gen(1,1.34);   
        std::uniform_real_distribution<double> resttime_gen(0.1,2);  
        sfm::pos2d origin = {origin_gen_x(generator),origin_gen_y(generator)};
        sfm::pos2d destination = {destination_gen_x(generator), destination_gen_y(generator)};
        sfm::dir2d velocity = {velocity_gen(generator), velocity_gen(generator)};
        sfm::pos2d current_position = origin;
        double speed = speed_gen(generator);
        double rest_time = resttime_gen(generator);
        sfm::Forces ped_spawn(origin,destination,velocity,current_position,speed,rest_time);
        std::shared_ptr<sfm::Forces> pointer(new sfm::Forces(ped_spawn));
        pedestrians.emplace_back(pointer);
    }

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
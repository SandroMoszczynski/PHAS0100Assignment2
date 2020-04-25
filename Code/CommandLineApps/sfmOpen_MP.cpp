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
#include <fstream>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#endif

std::vector<std::shared_ptr<sfm::Forces> > &Update_Pedestrian(std::vector<std::shared_ptr<sfm::Forces> > &pedestrians, double dt, double finish_time_s)
{
    double v_max = 1.3;
    sfm::dir2d temp_force;
    sfm::dir2d new_velocity;
    sfm::dir2d position;
    sfm::pos2d new_position;
    for(int t=0; t<(finish_time_s/dt);++t){
        #pragma omp tasks private(new_velocity),private(position),private(new_position), firstprivate(temp_force), shared(pedestrians)
        {
            #pragma omp for
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
    }
    return pedestrians;
}

std::vector<std::shared_ptr<sfm::Forces> > &Create_Pedestrian(std::vector<std::shared_ptr<sfm::Forces> > &pedestrians, int no_pedestrians)
{
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
    std::string choice;
    std::cout << "Default?" << std::endl;
    std::cin >> choice;
    double dt = 0.1; //s
    double finish_time_s;//second
    int no_pedestrians;
    if(choice ==  "Yes"){
        finish_time_s = 100;//second
        no_pedestrians = 1000;
    }
    else{
        std::cout << "Finish Time/0.1?" << std::endl;
        std::cin >> finish_time_s;
        std::cout << "No Pedestrians?" << std::endl;
        std::cin >> no_pedestrians;
    }
    
    std::ofstream Results;
    std::stringstream FileName;
    FileName << "Benchmarking_" << no_pedestrians << "_" << finish_time_s << ".txt";
    Results.open(FileName.str());
    Results << "Testing done with " << no_pedestrians << " pedestrians  \n"
            << "For time " << finish_time_s << " seconds at intervals of 0.1s\n";
    std::cout   << "Testing done with " << no_pedestrians << " pedestrians \n"
                << "For time " << finish_time_s << " seconds at intervals of 0.1s\n" << std::endl;

    // done without multi threading 
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians1;
    std::cout << "start no mp" << std::endl;
    std::clock_t c_start1 = std::clock();
    auto t_start1 = std::chrono::high_resolution_clock::now();
    pedestrians1 = Create_Pedestrian(pedestrians1,no_pedestrians);
    pedestrians1 = Update_Pedestrian(pedestrians1,dt,finish_time_s);
    std::clock_t c_end1 = std::clock();
    auto t_end1 = std::chrono::high_resolution_clock::now();
    std::cout << "stop no mp" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "CPU time used no mp: "
            << 1000.0 * (c_end1-c_start1) / CLOCKS_PER_SEC << " ms\n"
            << "Wall clock time passed no mp: "
            << std::chrono::duration<double, std::milli>(t_end1-t_start1).count()
            << " ms\n";
    Results << "Thread Count;   CPU time(ms):      Wall clock time(ms): \n"
            << std::fixed << std::setprecision(2)
            << "1, "
            << 1000.0 * (c_end1-c_start1) / CLOCKS_PER_SEC << " ," 
            << std::chrono::duration<double, std::milli>(t_end1-t_start1).count() <<"\n";

    //now iterate with incresing number of thread with max number for each pc

    int max_threads = omp_get_max_threads(); // throws an error on vscode but works

    for(int num_threads = 1;num_threads<(max_threads+1);++num_threads)
    {
        std::cout << "Threadcount: " << num_threads << std::endl;
        #ifdef _OPENMP
        omp_set_num_threads(num_threads);
        #endif

        std::vector<std::shared_ptr<sfm::Forces> >pedestrians;


        std::cout << "start with mp" << std::endl;
        std::clock_t c_start = std::clock();
        auto t_start = std::chrono::high_resolution_clock::now();
        pedestrians = Create_Pedestrian(pedestrians,no_pedestrians);
        sfm::pos2d origin_before = pedestrians[0]->Return_Origin();
        #pragma omp parallel shared(pedestrians)
        {
            #pragma single nowait
            {
                pedestrians = Update_Pedestrian(pedestrians,dt,finish_time_s);
            }
        }
        std::clock_t c_end = std::clock();
        auto t_end = std::chrono::high_resolution_clock::now();
        std::cout << "stop with mp" << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "CPU time used: "
                << 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC << " ms\n"
                << "Wall clock time passed: "
                << std::chrono::duration<double, std::milli>(t_end-t_start).count()
                << " ms\n";
        Results << std::fixed << std::setprecision(2)
                << num_threads << ", "
                << 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC << " ," 
                << std::chrono::duration<double, std::milli>(t_end-t_start).count() <<"\n";
    }
    Results.close();
    return 0;
}
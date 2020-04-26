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

//open mp initialisation
#ifdef _OPENMP
#include <omp.h>
#endif

//dummy function to allow us to set up open mp easier
std::vector<std::shared_ptr<sfm::Forces> > &Update_Pedestrian(std::vector<std::shared_ptr<sfm::Forces> > &pedestrians, double dt, double finish_time_s)
{
    // decided to leave v_max as default, but can change dt and finish time, also set up variables for open mp
    double v_max = 1.3;
    sfm::dir2d temp_force;
    sfm::dir2d new_velocity;
    sfm::dir2d position;
    sfm::pos2d new_position;

    //for loop over time
    for(int t=0; t<(finish_time_s/dt);++t){

        //set up open mp tasks with each private and shared components
        #pragma omp tasks private(new_velocity),private(position),private(new_position), firstprivate(temp_force), shared(pedestrians)
        {
            //set up parellel for loop
            #pragma omp for
            for(int j=0; j<pedestrians.size();++j){    

                //set up temp force as the resultant force of each pedestrian, and a new velocity calculated from it           
                temp_force = pedestrians[j]->Resultant_force(pedestrians,temp_force, dt);
                new_velocity = (temp_force*dt) + pedestrians[j]->Return_Velocity();

                //normalising new velocity speed
                if(new_velocity.length() > v_max*pedestrians[j]->Return_Speed()){
                    new_velocity = new_velocity*(v_max*pedestrians[j]->Return_Speed()/new_velocity.length());
                }

                //setting up position from each pedestrian to make the code less dense                
                position = {pedestrians[j]->Return_Current_Position()[1],pedestrians[j]->Return_Current_Position()[0]};
                new_position = {position[1]+(new_velocity[1]*dt),(position[0]+new_velocity[0]*dt)};

                //updating new positions at the end of loop
                pedestrians[j]->Update_Velocity(new_velocity);
                pedestrians[j]->Update_Current_Position(new_position);
            }
        }
    }
    return pedestrians;
}


//another dummy function that creates the pedestrian for easier implementation of open mp
std::vector<std::shared_ptr<sfm::Forces> > &Create_Pedestrian(std::vector<std::shared_ptr<sfm::Forces> > &pedestrians, int no_pedestrians)
{
    //variable initialisations for easier alteration, started in 2 boxes at left and right side of corridor
    sfm::dir2d left_side_x(1,1);
    sfm::dir2d left_side_y(0.1,9.9);
    sfm::dir2d right_side_x(48,49);
    sfm::dir2d right_side_y(0.1,9.9);
    sfm::dir2d direc1(1,0);
    sfm::dir2d direc2(-1,0);

    //this sets up the left pedestrians by appending each pedestrian created to the inputted vector
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians1;
    pedestrians1 = sfm::Factory::Directional(pedestrians1,no_pedestrians/2,direc1,left_side_x,left_side_y);
    for(int point = 0; point < pedestrians1.size();++point){
        pedestrians.emplace_back(pedestrians1[point]);
    } 

    //and this the right
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians2;
    pedestrians2 = sfm::Factory::Directional(pedestrians2,no_pedestrians/2,direc2,right_side_x,right_side_y);
    for(int point = 0; point < pedestrians2.size();++point){
        pedestrians.emplace_back(pedestrians2[point]);
    }

    //directional pedestrians were chosen as they keep moving for the whole time, makes the calculations consistent
    return pedestrians;   
}

//the main function
int main()
{
    //small input/output question to make doing multiple benchmarks easier
    std::string choice;
    std::cout << "Default?(Yes/No)" << std::endl;
    std::cin >> choice;

    //decided to keep dt constant as it keeps the calculations a little neater
    //otherwise this section is just initialising variables for later
    double dt = 0.1; 
    double finish_time_s;
    int no_pedestrians;
    if(choice ==  "Yes"){
        finish_time_s = 100;
        no_pedestrians = 1000;
    }
    else{
        std::cout << "Finish Time/0.1?" << std::endl;
        std::cin >> finish_time_s;
        std::cout << "No Pedestrians?" << std::endl;
        std::cin >> no_pedestrians;
    }
    
    //naming of file is automatic if you wanted to do multiple benchmarks
    std::ofstream Results;
    std::stringstream FileName;
    FileName << "Benchmarking_" << no_pedestrians << "_" << finish_time_s << ".txt";
    Results.open(FileName.str());

    //this gives a little preamble at the top of each file, and outputs it to the terminal
    Results << "Testing done with " << no_pedestrians << " pedestrians  \n"
            << "For time " << finish_time_s << " seconds at intervals of 0.1s\n";
    std::cout   << "Testing done with " << no_pedestrians << " pedestrians \n"
                << "For time " << finish_time_s << " seconds at intervals of 0.1s\n" << std::endl;

    //initialise a ptr vector for the non mp test
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians1;

    //start timing 
    std::cout << "start no mp" << std::endl;
    std::clock_t c_start1 = std::clock();
    auto t_start1 = std::chrono::high_resolution_clock::now();

    //create the pedestrians for the test
    pedestrians1 = Create_Pedestrian(pedestrians1,no_pedestrians);

    //update their position
    pedestrians1 = Update_Pedestrian(pedestrians1,dt,finish_time_s);

    //stop the timers
    std::clock_t c_end1 = std::clock();
    auto t_end1 = std::chrono::high_resolution_clock::now();
    std::cout << "stop no mp" << std::endl;

    // this outputs to the terminal the time elapsed
    std::cout << std::fixed << std::setprecision(2) << "CPU time used no mp: "
            << 1000.0 * (c_end1-c_start1) / CLOCKS_PER_SEC << " ms\n"
            << "Wall clock time passed no mp: "
            << std::chrono::duration<double, std::milli>(t_end1-t_start1).count()
            << " ms\n";

    //this inputs the same data as above into a csv format of the file
    Results << "Thread Count;   CPU time(ms):      Wall clock time(ms): \n"
            << std::fixed << std::setprecision(2)
            << "1, "
            << 1000.0 * (c_end1-c_start1) / CLOCKS_PER_SEC << " ," 
            << std::chrono::duration<double, std::milli>(t_end1-t_start1).count() <<"\n";

    // throws an error on vscode but works
    int max_threads = omp_get_max_threads(); 

    //now iterate with incresing number of thread with max number for each pc
    for(int num_threads = 1;num_threads<(max_threads+1);++num_threads)
    {
        //outputting how far along on loop we are
        std::cout << "Threadcount: " << num_threads << std::endl;

        //open mp initialisations
        #ifdef _OPENMP
        omp_set_num_threads(num_threads);
        #endif

        //initialising empty vector for pointer storage
        std::vector<std::shared_ptr<sfm::Forces> >pedestrians;

        // starting timer here
        std::cout << "start with mp" << std::endl;
        std::clock_t c_start = std::clock();
        auto t_start = std::chrono::high_resolution_clock::now();

        //creating pedestrians here
        pedestrians = Create_Pedestrian(pedestrians,no_pedestrians);

        //check for correct data output after parrellelisation
        // sfm::pos2d origin_before = pedestrians[0]->Return_Origin();

        //implementing open mp
        #pragma omp parallel shared(pedestrians)
        {
            #pragma single nowait
            {
                pedestrians = Update_Pedestrian(pedestrians,dt,finish_time_s);
            }
        }

        //stoping timers here
        std::clock_t c_end = std::clock();
        auto t_end = std::chrono::high_resolution_clock::now();

        //output to terminal to update on progress
        std::cout << "stop with mp" << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "CPU time used: "
                << 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC << " ms\n"
                << "Wall clock time passed: "
                << std::chrono::duration<double, std::milli>(t_end-t_start).count()
                << " ms\n";

        //exporting to text file for easy usage later
        Results << std::fixed << std::setprecision(2)
                << num_threads << ", "
                << 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC << " ," 
                << std::chrono::duration<double, std::milli>(t_end-t_start).count() <<"\n";

        //section to see if variables are the same before as after, when done on my pc tests came out as 0 difference
        // sfm::pos2d origin_after = pedestrians[0]->Return_Origin();
        // std::cout << origin_before[0] - origin_after[0] << " ," << origin_before[1] - origin_after[1] << std::endl;
        //presumably if the data hasnt been garbled it should all be correct without needing to check each entry
    }

    //closes the txt file
    Results.close();
    return 0;
}
#include "catch.hpp"
#include "sfmCatchMain.h"
#include "sfmPedestrianSpawner.h"
#include <iostream>
#include <vector>

//this is the chosen file for unit tests

namespace sfm{

TEST_CASE( "unit tests for pedestrian class", "[Pedestrian Class]") {

    //pedestrian 1, note y values over 10
    pos2d origin(45,215);
    pos2d destination(45,893);
    dir2d velocity(42,42);
    pos2d current_position(19,84);
    double speed = 145;
    double rest_time = 12;

    //create test variables to test against
    pos2d test_origin;
    pos2d test_destination;
    dir2d test_velocity;
    pos2d test_current_position;
    double test_speed;
    double test_rest_time;

    //create the pedestrian to test
    Pedestrian test1(origin, destination,velocity, current_position, speed, rest_time);
    test_origin = test1.Return_Origin();
    test_destination = test1.Return_Destination();
    test_velocity = test1.Return_Velocity();
    test_current_position = test1.Return_Current_Position();
    test_speed = test1.Return_Speed();
    test_rest_time = test1.Return_Rest_time();

    //these check that the classes have been instantiated correctly,
    //and that the pos2d's overlap correctly and that dir2d's dont
    REQUIRE( test_origin[0] == 5);
    REQUIRE( test_origin[1] == 45);
    REQUIRE( test_destination[0] == 3);
    REQUIRE( test_destination[1] == 45);
    REQUIRE( test_velocity[0] == 42);
    REQUIRE( test_velocity[1] == 42);
    REQUIRE( test_current_position[0] == 4);
    REQUIRE( test_current_position[1] == 19);
    REQUIRE( test_speed == 145);
    REQUIRE( test_rest_time == 12);
}

TEST_CASE( "unit tests for forces class", "[Forces Class]") {

    //pedestrian 1
    sfm::pos2d origin1(19,2); 
    sfm::pos2d destination1(14,1);
    sfm::dir2d velocity1(1,0.5);
    sfm::pos2d current_position1(14,8);
    double speed1 = 1.5;
    double rest_time1 = 1;

    //pedestrian 2
    sfm::pos2d origin2(1,6);
    sfm::pos2d destination2(14,9.5);
    sfm::dir2d velocity2(1.5,0.5);
    sfm::pos2d current_position2(14,7);
    double speed2 = 1.5;
    double rest_time2 = 0.5;

    //pedestrian 3
    sfm::pos2d origin3(4,5); 
    sfm::pos2d destination3(44,9);
    sfm::dir2d velocity3(2,0.5);
    sfm::pos2d current_position3(34,1);
    double speed3 = 1;
    double rest_time3 = 0.5;

    //creating classes through Forces
    sfm::Forces test2_1(origin1, destination1,velocity1, current_position1, speed1, rest_time1);
    sfm::Forces test2_2(origin2, destination2,velocity2, current_position2, speed2, rest_time2);
    sfm::Forces test2_3(origin3, destination3,velocity3, current_position3, speed3, rest_time3);

    //making a pointer vector of classes, unecessery but will be used
    // in later parts
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians_test;
    std::shared_ptr<sfm::Forces> test_ptr1(new sfm::Forces(test2_1));
    std::shared_ptr<sfm::Forces> test_ptr2(new sfm::Forces(test2_2));
    std::shared_ptr<sfm::Forces> test_ptr3(new sfm::Forces(test2_3));
    pedestrians_test.emplace_back(test_ptr1);
    pedestrians_test.emplace_back(test_ptr2);
    pedestrians_test.emplace_back(test_ptr3);

    //setting up values for testing
    dir2d test_desired_direction = pedestrians_test[0]->desired_direction(test_desired_direction);
    dir2d test_attractive_force = pedestrians_test[0]->attractive_force(test_attractive_force);
    double test_elipse_dif = pedestrians_test[0]->elipse(pedestrians_test[1],test_elipse_dif);
    double test_elipse_same = pedestrians_test[0]->elipse(pedestrians_test[0],test_elipse_same);
    dir2d test_rep_force_diff = pedestrians_test[0]->repulsive_force(pedestrians_test[1],test_rep_force_diff);
    dir2d test_rep_force_same = pedestrians_test[0]->repulsive_force(pedestrians_test[0],test_rep_force_same);
    double test_fov_in = pedestrians_test[0]->fov(test_rep_force_diff,test_desired_direction,test_fov_in);
    double test_fov_out = pedestrians_test[2]->fov(test_rep_force_same,test_desired_direction,test_fov_out);
    dir2d test_border_repulsive_top = pedestrians_test[0]->border_repulsive(test_border_repulsive_top);
    dir2d test_border_repulsive_mid = pedestrians_test[1]->border_repulsive(test_border_repulsive_mid);
    dir2d test_border_repulsive_bot = pedestrians_test[2]->border_repulsive(test_border_repulsive_bot);
    dir2d test_resultant = pedestrians_test[0]->Resultant_force(pedestrians_test,test_resultant);

    //set up require tests, note the border repulsives check that each of them points in
    //the correct direction and that the fov works the the two pedestrians facing each
    //other but not the other two.
    REQUIRE( test_desired_direction[0] != 0);
    REQUIRE( test_attractive_force[1] != 0);
    REQUIRE( test_elipse_dif != 3);
    REQUIRE( test_elipse_same == 0);
    REQUIRE( test_rep_force_diff[0] != 0);
    REQUIRE( test_rep_force_same[0] == 0);
    REQUIRE( test_fov_in == 0.5);
    REQUIRE( test_fov_out == 1);
    REQUIRE( test_border_repulsive_top[0] < 0);
    REQUIRE( test_border_repulsive_mid[0] != 0);
    REQUIRE( test_border_repulsive_bot[0] > 0);
    REQUIRE( test_resultant[1] != 0);
}

TEST_CASE( "unit tests for Factory class", "[Factory Class]") {

    //test for creating dir2d
    sfm::dir2d test_R_Dir2d;
    test_R_Dir2d = sfm::Factory::R_Dir2d(test_R_Dir2d,2,2,1,2);

    //test for creating pos2d
    sfm::pos2d test_R_Pos2d;
    test_R_Pos2d = sfm::Factory::R_Pos2d(test_R_Pos2d,52,52,1,2);

    //test for creating double
    double test_R_Doub;
    test_R_Doub = sfm::Factory::R_Doub(test_R_Doub,5,5);

    //test for creating random spawned(), number spawned and check they are all
    //within the given coordinates
    std::vector<std::shared_ptr<sfm::Forces> >test_ped_spawner;
    test_ped_spawner = sfm::Factory::Spawner(test_ped_spawner,100);
    int error_count = 0;
    for(int i = 0; i<test_ped_spawner.size();++i){
        if(test_ped_spawner[i]->Return_Current_Position()[1] != test_ped_spawner[i]->Return_Origin()[1]){
            error_count += 1;
        }
        if(test_ped_spawner[i]->Return_Current_Position()[0] != test_ped_spawner[i]->Return_Origin()[0]){
            error_count += 1;
        }
        if(test_ped_spawner[i]->Return_Origin()[1] < 0.1 || test_ped_spawner[i]->Return_Origin()[1] > 49.9){
            error_count += 1;
        }
        if(test_ped_spawner[i]->Return_Origin()[0] < 0.1 || test_ped_spawner[i]->Return_Origin()[0] > 9.9){
            error_count += 1;
        }
        if(test_ped_spawner[i]->Return_Velocity()[1] < -1.5 || test_ped_spawner[i]->Return_Velocity()[1] > 1.5){
            error_count += 1;
        }
        if(test_ped_spawner[i]->Return_Velocity()[0] < -1.5 || test_ped_spawner[i]->Return_Velocity()[0] > 1.5){
            error_count += 1;
        }
        if(test_ped_spawner[i]->Return_Destination()[1] < 0.1 || test_ped_spawner[i]->Return_Destination()[1] > 49.9){
            error_count += 1;
        }
        if(test_ped_spawner[i]->Return_Destination()[0] < 0.1 || test_ped_spawner[i]->Return_Destination()[0] > 9.9){
            error_count += 1;
        }    
        if(test_ped_spawner[i]->Return_Speed() < 0.1 || test_ped_spawner[i]->Return_Speed() > 1.5){
            error_count += 1;
        } 
        if(test_ped_spawner[i]->Return_Rest_time() < 0.1 || test_ped_spawner[i]->Return_Rest_time() > 1.5){
            error_count += 1;
        }            
    }

    //test for creating uniform(), and test for uniform spread
    std::vector<std::shared_ptr<sfm::Forces> >test_ped_uniform;
    sfm::dir2d test_dest_x(1,2);
    sfm::dir2d test_dest_y(0.1,9.9);  
    test_ped_uniform = sfm::Factory::Uniform(test_ped_uniform,"Targeted",100,test_dest_x,test_dest_y);
    double origin_dist_x = 0;
    double origin_dist_y = 0;
    double velocity_x = 0;
    double velocity_y = 0;
    double speed = 0;
    for(int j=0;j<test_ped_uniform.size();++j){
        origin_dist_x += test_ped_uniform[j]->Return_Origin()[1];
        origin_dist_y += test_ped_uniform[j]->Return_Origin()[0];
        velocity_x += test_ped_uniform[j]->Return_Velocity()[1];
        velocity_y += test_ped_uniform[j]->Return_Velocity()[0];   
        speed +=  test_ped_uniform[j]->Return_Speed();
    }

    //test for creating distributed(), set at one side
    std::vector<std::shared_ptr<sfm::Forces> >test_ped_dis_dir;
    sfm::dir2d test_start_pos_x(1,1);
    sfm::dir2d test_start_pos_y(0.1,9.9);
    sfm::dir2d test_direction(1,0);
    test_ped_dis_dir = sfm::Factory::Distributed(test_ped_dis_dir,"Directional",10,test_direction,test_direction,test_start_pos_x,test_start_pos_y);
    double y_distribution = 0; 
    for(int j=0;j<test_ped_dis_dir.size();++j){
        y_distribution += test_ped_dis_dir[j]->Return_Origin()[0];
    }

    //test that directional moves along a straight line
    std::vector<std::shared_ptr<sfm::Forces> >test_ped_directional;
    test_ped_directional = sfm::Factory::Directional(test_ped_directional,1,{1,0},{1,1},{5,5},{1,1},{1,1});
    double dt = 1;
    for(int t=0; t<(10/dt);++t){
        sfm::dir2d temp_force = test_ped_directional[0]->Resultant_force(test_ped_directional,temp_force, dt);
        sfm::dir2d  new_velocity = (temp_force*dt) + test_ped_directional[0]->Return_Velocity();
        if(new_velocity.length() > 1.3*test_ped_directional[0]->Return_Speed()){
            new_velocity = new_velocity*(1.3*test_ped_directional[0]->Return_Speed()/new_velocity.length());
            }
        sfm::dir2d position(test_ped_directional[0]->Return_Current_Position()[1],test_ped_directional[0]->Return_Current_Position()[0]);
        sfm::pos2d new_position = {position[1]+(new_velocity[1]*dt),(position[0]+new_velocity[0]*dt)};
        test_ped_directional[0]->Update_Velocity(new_velocity);
        test_ped_directional[0]->Update_Current_Position(new_position);   
    }

    //these test to make sure that the random generators worked as intended
    REQUIRE(test_R_Dir2d[1] == 2);
    REQUIRE(test_R_Dir2d[0] < 2);
    REQUIRE(test_R_Dir2d[0] > 1);
    REQUIRE(test_R_Pos2d[1] == 2);
    REQUIRE(test_R_Pos2d[0] < 2);
    REQUIRE(test_R_Pos2d[0] > 1);
    REQUIRE(test_R_Doub == 5);

    //these check that the correct ammount has been spawned, and that none of
    //the pedestrians are out of the scope specified
    REQUIRE(test_ped_spawner.size() == 100);
    REQUIRE(error_count == 0);

    //these check the distribution is random. ie x of 49.9-0.1 = 49.8. 49.8/2 = 24.4
    //and then a range of 5 for random chance. the same for the y values
    REQUIRE(origin_dist_x/100-24.4 < 5);
    REQUIRE(origin_dist_y/100-4.9 < 1);

    //these two are from -1.5 to 1.5 so should havea  good spread close to 0
    REQUIRE(velocity_x/100 == 0);
    REQUIRE(velocity_y/100 == 0);

    //calculated as above
    REQUIRE(speed/100-0.7 < 0.2);

    //checks created at x=1 and that y distribution is random
    REQUIRE(test_ped_dis_dir[0]->Return_Origin()[1] == 1);
    REQUIRE(y_distribution/100-4.9 < 1);

    //checks it moved in a straight line, note it starts at 1 and moves for 10 so it should be about 11
    REQUIRE(test_ped_directional[0]->Return_Current_Position()[1]-11 < 0.001);
    REQUIRE(test_ped_directional[0]->Return_Current_Position()[0] == 5);
}
}
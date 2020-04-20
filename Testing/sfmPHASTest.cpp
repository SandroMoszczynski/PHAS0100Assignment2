#include "catch.hpp"
#include "sfmCatchMain.h"
#include "sfmForces.h"
#include <iostream>
#include <vector>

namespace sfm{

TEST_CASE( "unit tests for pedestrian class", "[Pedestrian Class]") {
    pos2d origin(45,215);
    pos2d destination(45,893);
    dir2d velocity(42,42);
    pos2d current_position(19,84);
    double speed = 145;
    double rest_time = 12;
    pos2d test_origin;
    pos2d test_destination;
    dir2d test_velocity;
    pos2d test_current_position;
    double test_speed;
    double test_rest_time;
    Pedestrian test1(origin, destination,velocity, current_position, speed, rest_time);
    test_origin = test1.Return_Origin();
    test_destination = test1.Return_Destination();
    test_velocity = test1.Return_Velocity();
    test_current_position = test1.Return_Current_Position();
    test_speed = test1.Return_Speed();
    test_rest_time = test1.Return_Rest_time();
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
    //making a pointer vector of classes
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
    //set up require tests
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

}
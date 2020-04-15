#include "catch.hpp"
#include "sfmCatchMain.h"
#include "sfmPedestrian.h"
#include <iostream>
#include <vector>

namespace sfm{

TEST_CASE( "create a class for DataImporter", "[Linear Data Creator]") {
    vec2d origin(45,215);
    vec2d destination(45,893);
    vec2d velocity(42,42);
    vec2d current_position(19,84);
    double speed = 145;
    double rest_time = 12;
    sfm::vec2d test_origin;
    sfm::vec2d test_destination;
    sfm::vec2d test_velocity;
    sfm::vec2d test_current_position;
    double test_speed;
    double test_rest_time;
    Pedestrians test1(origin, destination,velocity, current_position, speed, rest_time);
    test_origin = test1.Return_Origin();
    test_destination = test1.Return_Destination();
    test_velocity = test1.Return_Velocity();
    test_current_position = test1.Return_Current_Position();
    test_speed = test1.Return_Speed();
    test_rest_time = test1.Return_Rest_time();
    REQUIRE( test_origin[0] == 215);
    REQUIRE( test_origin[1] == 45);
    REQUIRE( test_destination[0] == 893);
    REQUIRE( test_destination[1] == 45);
    REQUIRE( test_velocity[0] == 42);
    REQUIRE( test_velocity[1] == 42);
    REQUIRE( test_current_position[0] == 84);
    REQUIRE( test_current_position[1] == 19);
    REQUIRE( test_speed == 145);
    REQUIRE( test_rest_time == 12);
}

}
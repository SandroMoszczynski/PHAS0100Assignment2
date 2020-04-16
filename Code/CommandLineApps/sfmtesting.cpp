#include <iostream>
#include <vector>
#include "sfmMyFunctions.h"
#include "sfmExceptionMacro.h"
#include "sfmBasicTypes.h"
#include "sfmPedestrian.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

int main()
{
    sfm::pos2d origin(-51,12); //b/w 50 and 10
    sfm::pos2d destination(15,83);
    sfm::dir2d velocity(1865,-0.3354);
    sfm::pos2d current_position(0,0);
    double speed = 145;
    double rest_time = 0;
    sfm::Pedestrians test1(origin, destination,velocity, current_position, speed, rest_time);
    double test_speed;
    sfm::pos2d test_origin;
    sfm::pos2d test_destination;
    sfm::dir2d test_velocity;
    sfm::pos2d test_current_position(0,0);
    test_speed = test1.Return_Speed();
    test_origin = test1.Return_Origin();
    std::cout << origin[1] << std::endl;
    std::cout << origin[0] << std::endl;
    return 0;
}
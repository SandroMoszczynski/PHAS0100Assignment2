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
    sfm::vec2d origin(14,0);
    sfm::vec2d destination(0,0);
    sfm::vec2d velocity(0,0);
    sfm::vec2d current_position(0,0);
    double speed = 145;
    double rest_time = 0;
    sfm::Pedestrians test1(origin, destination,velocity, current_position, speed, rest_time);
    double test_speed;
    test_speed = test1.Return_Speed();
    std::cout << test_speed << std::endl;
    
    return 0;
}
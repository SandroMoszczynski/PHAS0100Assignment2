#include <iostream>
#include <vector>
#include "sfmMyFunctions.h"
#include "sfmExceptionMacro.h"
#include "sfmBasicTypes.h"
#include "sfmForces.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

int main()
{
    sfm::pos2d origin1(19,2); //b/w 50 and 10
    sfm::pos2d destination1(16.285,2.89);
    sfm::dir2d velocity1(1.65,0.3354);
    sfm::pos2d current_position1(3,9.9);
    double speed1 = 1.15;
    double rest_time1 = 1;
    sfm::pos2d origin2(1,6); //b/w 50 and 10
    sfm::pos2d destination2(46.50,1.648);
    sfm::dir2d velocity2(1.56,0.354);
    sfm::pos2d current_position2(5,9.9);
    double speed2 = 1.55;
    double rest_time2 = 0.5;
    sfm::pos2d origin3(4,5); //b/w 50 and 10
    sfm::pos2d destination3(1,83);
    sfm::dir2d velocity3(185,-0.3354);
    sfm::pos2d current_position3(4,9.9);
    double speed3 = 5;
    double rest_time3 = 0.4;
    //sfm::Pedestrian test1(origin1, destination1,velocity1, current_position1, speed1, rest_time1);
    sfm::Forces test1(origin1, destination1,velocity1, current_position1, speed1, rest_time1);
    sfm::Forces test2(origin2, destination2,velocity2, current_position2, speed2, rest_time3);
    sfm::Forces test3(origin3, destination3,velocity3, current_position3, speed3, rest_time3);
    //now i create all the tests variables
    sfm::dir2d desdirec;
    sfm::dir2d repforce;
    sfm::dir2d ataforce;
    sfm::dir2d ataforce2;
    std::shared_ptr<sfm::Forces> sp1(new sfm::Forces(test1));
    std::shared_ptr<sfm::Forces> sp2(new sfm::Forces(test2));
    std::shared_ptr<sfm::Forces> sp3(new sfm::Forces(test3));
    double elipse_test;
    elipse_test = test1.elipse(sp1,sp2,elipse_test);
    //std::shared_ptr<sfm::Pedestrian> sp3(new sfm::Pedestrian(test3));
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    pedestrians.emplace_back(sp1);
    pedestrians.emplace_back(sp2);
    pedestrians.emplace_back(sp3);
    //repforce = sp1->border_repulsive(repforce);
    sfm::dir2d repforce1 = pedestrians[0]->border_repulsive(repforce1);
    sfm::dir2d repforce2 = pedestrians[1]->border_repulsive(repforce2);
    sfm::dir2d repforce3 = pedestrians[2]->border_repulsive(repforce3);
    desdirec = test1.desired_direction(desdirec);
    ataforce = test2.attractive_force(ataforce);
    repforce = test1.repulsive_force(sp1,sp2,elipse_test,repforce);
    //std::cout << sp1.get() << std::endl;
    //std::cout << sp2.get() << std::endl;
    //std::cout << sp3.get() << std::endl;
    double test_speed;
    sfm::pos2d test_origin;
    sfm::pos2d test_destination;
    sfm::dir2d test_velocity;
    sfm::pos2d test_current_position(0,0);
    // need to figure out how to get multiple pedestrians into one vector, maybe try working with a different method
    //test_speed = force1.Return_Speed();
    //test_speed = sp1->Return_Speed();
    //std::cout << test_speed << std::endl;
    //std::cout << elipse_test << std::endl;
    std::cout << repforce1[1] << std::endl;
    std::cout << repforce1[0] << std::endl;
    std::cout << repforce2[1] << std::endl;
    std::cout << repforce2[0] << std::endl;
    std::cout << repforce3[1] << std::endl;
    std::cout << repforce3[0] << std::endl;
    //std::cout << ataforce2[1] << std::endl;
    //std::cout << ataforce2[0] << std::endl;
    return 0;
}
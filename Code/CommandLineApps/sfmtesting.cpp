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
    sfm::pos2d current_position1(46,6);
    double speed1 = 1.15;
    double rest_time1 = 1;
    sfm::pos2d origin2(1,6); //b/w 50 and 10
    sfm::pos2d destination2(46.50,1.648);
    sfm::dir2d velocity2(1.56,0.354);
    sfm::pos2d current_position2(15,5);
    double speed2 = 1.55;
    double rest_time2 = 0.5;
    sfm::pos2d origin3(4,5); //b/w 50 and 10
    sfm::pos2d destination3(23,9);
    sfm::dir2d velocity3(1.8,0.3354);
    sfm::pos2d current_position3(4,3);
    double speed3 = 1.3;
    double rest_time3 = 0.4;
    //sfm::Pedestrian test1(origin1, destination1,velocity1, current_position1, speed1, rest_time1);
    sfm::Forces test1(origin1, destination1,velocity1, current_position1, speed1, rest_time1);
    sfm::Forces test2(origin2, destination2,velocity2, current_position2, speed2, rest_time2);
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
    //elipse_test = test1.elipse(sp2,elipse_test);
    //std::shared_ptr<sfm::Pedestrian> sp3(new sfm::Pedestrian(test3));
    std::vector<std::shared_ptr<sfm::Forces> >pedestrians;
    pedestrians.emplace_back(sp1);
    pedestrians.emplace_back(sp2);
    pedestrians.emplace_back(sp3);
    //std::cout << pedestrians.size() << std::endl;
    //repforce = sp1->border_repulsive(repforce);
    //sfm::dir2d repforce1 = pedestrians[0]->border_repulsive(repforce1);
    //sfm::dir2d repforce2 = test1.repulsive_force(pedestrians[2],repforce2);
    //sfm::dir2d repforce3 = pedestrians[2]->border_repulsive(repforce3);
    //desdirec = test1.desired_direction(desdirec);
    //ataforce = test2.attractive_force(ataforce);
    //repforce = test1.repulsive_force(pedestrians[1],repforce);
    sfm::dir2d totalforce = test2.Resultant_force(pedestrians,totalforce);
    //std::cout << sp1.get() << std::endl;
    //std::cout << sp2.get() << std::endl;
    //std::cout << sp3.get() << std::endl;
    double test_speed;
    sfm::pos2d test_origin;
    sfm::pos2d test_destination;
    sfm::dir2d test_velocity;
    sfm::pos2d test_current_position(0,0);
    //test_speed = force1.Return_Speed();
    //test_speed = sp1->Return_Speed();
    //std::cout << test_speed << std::endl;
    //std::cout << elipse_test << std::endl;
    std::cout << totalforce[1] << std::endl;
    std::cout << totalforce[0] << std::endl;
    //std::cout << repforce2[1] << std::endl;
    //std::cout << repforce2[0] << std::endl;
    // std::cout << repforce3[1] << std::endl;
    // std::cout << repforce3[0] << std::endl;
    //std::cout << ataforce2[1] << std::endl;
    //std::cout << ataforce2[0] << std::endl;
    return 0;
}
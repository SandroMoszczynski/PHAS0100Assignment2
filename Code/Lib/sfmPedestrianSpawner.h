#include "sfmForces.h"
#include <iostream>

namespace sfm{

class Factory : public Forces {
    public:
    static dir2d &R_Dir2d(dir2d &dir2d_out,
                    double x_min,
                    double x_max,
                    double y_min,
                    double y_max);  
    static pos2d &R_Pos2d(pos2d &pos2d_out,
                    double x_min, 
                    double x_max, 
                    double y_min, 
                    double y_max);
    static double &R_Doub(double &double_out, double double_min, double double_max);
    static std::vector<std::shared_ptr<sfm::Forces>> &Spawner(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        int no_pedestrians,
        double vel_min_x = -1.5,
        double vel_max_x = 1.5,
        double vel_min_y = -1.5,
        double vel_max_y = 1.5,
        double speed_min = 0.1,
        double speed_max = 1.5,
        double x_min_start = 0.1, 
        double x_max_start = 49.9, 
        double y_min_start = 0.1, 
        double y_max_start = 9.9,
        double x_min_dest = 0.1, 
        double x_max_dest = 49.9, 
        double y_min_dest = 0.1, 
        double y_max_dest = 9.9, 
        double rest_min = 0.1,
        double rest_max = 0.5
    ); //left defaults to make it easy to just spawn random pedestrians
    static std::vector<std::shared_ptr<sfm::Forces>> &Targeted(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        int no_pedestrians,
        dir2d Destination_x,
        dir2d Destination_y,
        dir2d x_values = {0.1,49.9}, 
        dir2d y_values = {0.1,9.9},
        dir2d speed = {0.1,1.5}, //min,max
        dir2d rest = {0.1,0.5} // min,max   
    );
    static std::vector<std::shared_ptr<sfm::Forces>> &Directional(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        int no_pedestrians,
        dir2d Direction,
        dir2d x_values = {0.1,49.9},
        dir2d y_values = {0.1,9.9},
        dir2d speed = {0.1,1.5},
        dir2d rest = {0.1,0.5}
    );
    static std::vector<std::shared_ptr<sfm::Forces>> &Uniform(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        std::string Type_pedestrian,
        int no_pedestrians,
        dir2d Destination_x,
        dir2d Destination_y
    ); //Type_pedestrian = Targeted or Directional
    static std::vector<std::shared_ptr<sfm::Forces>> &Distributed(
        std::vector<std::shared_ptr<sfm::Forces> > &pedestrians,
        std::string Type_pedestrian,
        int no_pedestrians,
        dir2d Destination_x,
        dir2d Destination_y,
        dir2d x_values,
        dir2d y_values
    );
};
}
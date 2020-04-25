#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "sfmBasicTypes.h"
#include "sfmPedestrian.h"

//for description see forces.cpp, note the default values and phi = 3.49 rad = 200 degrees 
//i found it easier to reinitialise a force and have it have all of the pedestrian class and
//force class methods. 

namespace sfm{

class Forces : public Pedestrian {
    public:
    Forces(pos2d origin,pos2d destination, dir2d velocity,pos2d current_position,double speed,double rest_time)
    :Pedestrian(origin,destination,velocity,current_position,speed,rest_time){};
    dir2d &desired_direction(dir2d &final_position);
    dir2d &attractive_force(dir2d &force);
    double &elipse(std::shared_ptr<Forces>pedesb, double &elipse, double delta_t = 2); // passa is the tested one, passb changes as all other pedestrians
    dir2d &repulsive_force(std::shared_ptr<Forces>pedesb, dir2d &Forces, double delta_t = 2);
    double &fov(dir2d ata_force,dir2d des_dir, double &fov, double phi = 3.49066,double c = 0.5);    
    dir2d &border_repulsive(dir2d &Forces);
    dir2d &Resultant_force(std::vector<std::shared_ptr<sfm::Forces> >Pedestrians, dir2d &Forces, double delta_t = 2);
};
}
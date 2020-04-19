#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "sfmBasicTypes.h"
#include "sfmPedestrian.h"

namespace sfm{


class Forces : public Pedestrian {
    public:
    Forces(pos2d origin,pos2d destination, dir2d velocity,pos2d current_position,double speed,double rest_time)
    :Pedestrian(origin,destination,velocity,current_position,speed,rest_time){};
    dir2d &desired_direction(dir2d &final_position);
    vec2d &attractive_force(vec2d &force);
    double &elipse(std::shared_ptr<Forces>pedesa, std::shared_ptr<Forces>pedesb, double &elipse, double step_width = 1.25); // passa is the tested one, passb changes as all other pedestrians
    //sfm::dir2d &repulsive_force();
    //sfm::dir2d &border_repulsive();
    //std::vector<shared_ptr<sfm::Pedestrians> > &Resultant_force();
};
}
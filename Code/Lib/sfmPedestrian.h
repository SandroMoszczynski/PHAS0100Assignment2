#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "sfmBasicTypes.h"

namespace sfm{

class Pedestrian {
    private:
    pos2d origin;
    pos2d destination;
    dir2d velocity;
    pos2d current_position;
    double speed;
    double rest_time;
    public:
    Pedestrian(pos2d origin,pos2d destination, dir2d velocity,pos2d current_position,double speed,double rest_time);
    pos2d &Return_Origin();
    pos2d &Return_Destination();
    dir2d &Return_Velocity();
    pos2d &Return_Current_Position();
    double &Return_Speed();
    double &Return_Rest_time();
    dir2d &Update_Velocity(dir2d new_velocity);
    pos2d &Update_Current_Position(pos2d new_pos);    
    };

}
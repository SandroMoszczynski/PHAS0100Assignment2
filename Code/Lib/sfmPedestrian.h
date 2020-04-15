#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "sfmBasicTypes.h"

namespace sfm{

class Pedestrians {
    private:
    vec2d origin;
    vec2d destination;
    vec2d velocity;
    vec2d current_position;
    double speed;
    double rest_time;
    public:
    Pedestrians(vec2d origin,vec2d destination, vec2d velocity,vec2d current_position,double speed,double rest_time);
    vec2d &Return_Origin();
    vec2d &Return_Destination();
    vec2d &Return_Velocity();
    vec2d &Return_Current_Position();
    double &Return_Speed();
    double &Return_Rest_time();
    };

}
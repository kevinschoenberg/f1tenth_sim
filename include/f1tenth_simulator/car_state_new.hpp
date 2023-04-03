#pragma once

namespace racecar_simulator {

struct CarStateNew {
    double x; // x position // x1
    double y; // y position // x2
    double theta; // orientation // x5
    double velocity; // x4
    double steer_angle; // x3
    double angular_velocity; // x6
    double slip_angle; // x7

    double angular_speed_f; //x8
    double angular_speed_r; //x9
    bool st_dyn;
};

}

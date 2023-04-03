#pragma once

#include "f1tenth_simulator/car_state_new.hpp"
#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"

namespace racecar_simulator {

class STDKinematics {

public:

    static CarStateNew update(
            const CarStateNew start,
            double accel,
            double steer_angle_vel,
            CarParams p,
            double dt);


    static CarStateNew update_k(
            const CarStateNew start,
            double accel,
            double steer_angle_vel,
            CarParams p,
            double dt);
};

}

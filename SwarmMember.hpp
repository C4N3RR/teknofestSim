#pragma once

#include "Common.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <Eigen/Dense>
#include <memory>
#include <iostream>

class SwarmMember {
public:
    SwarmMember(std::shared_ptr<mavsdk::System> system, int id);

    bool prepare_for_mission();

    void update_target(Eigen::Vector3d target_ned, float yaw_deg);

    void land();

    mavsdk::Telemetry::Position get_current_pos();

    int get_id() const { return _id; }

private:
    std::shared_ptr<mavsdk::System> _system;
    
    std::unique_ptr<mavsdk::Offboard> _offboard;
    std::unique_ptr<mavsdk::Action> _action;
    std::unique_ptr<mavsdk::Telemetry> _telemetry;
    
    int _id;
};
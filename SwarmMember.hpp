#pragma once

#include "Common.hpp" // Takımın ortak struct'ları (varsa)
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <Eigen/Dense>
#include <memory>
#include <iostream>

class SwarmMember {
public:
    // Kurucu fonksiyon (Senin ConnectionHandler'dan göndereceğin system objesini alır)
    SwarmMember(std::shared_ptr<mavsdk::System> system, int id);

    // Arm() -> Takeoff() -> Offboard moduna geçiş
    bool prepare_for_mission();

    // SwarmBrain'den gelen hedefleri (NED formatında) MAVSDK'ya basar
    void update_target(Eigen::Vector3d target_ned, float yaw_deg);

    // Acil durum veya iniş için
    void land();

    // Telemetri verisi çekmek için
    mavsdk::Telemetry::Position get_current_pos();

    // İHA ID'sini döndürmek için yardımcı fonksiyon
    int get_id() const { return _id; }

private:
    std::shared_ptr<mavsdk::System> _system;
    
    // MAVSDK eklentileri (Plugins)
    std::unique_ptr<mavsdk::Offboard> _offboard;
    std::unique_ptr<mavsdk::Action> _action;
    std::unique_ptr<mavsdk::Telemetry> _telemetry;
    
    int _id;
};

#include "SwarmMember.hpp"
#include <chrono>
#include <thread>

SwarmMember::SwarmMember(std::shared_ptr<mavsdk::System> system, int id) 
    : _system(system), _id(id) 
{
    _offboard = std::make_unique<mavsdk::Offboard>(_system);
    _action = std::make_unique<mavsdk::Action>(_system);
    _telemetry = std::make_unique<mavsdk::Telemetry>(_system);
}

bool SwarmMember::prepare_for_mission() {
    std::cout << "[İHA-" << _id << "] Görev hazırlığı başlıyor...\n";

    while (!_telemetry->health_all_ok()) {
        std::cout << "[İHA-" << _id << "] Sistem sağlığı bekleniyor (GPS vb.)...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    mavsdk::Action::Result arm_result = _action->arm();
    if (arm_result != mavsdk::Action::Result::Success) {
        std::cerr << "[İHA-" << _id << "] Arm BAŞARISIZ: " << arm_result << '\n';
        return false;
    }
    std::cout << "[İHA-" << _id << "] Arm başarılı.\n";

    mavsdk::Action::Result takeoff_result = _action->takeoff();
    if (takeoff_result != mavsdk::Action::Result::Success) {
        std::cerr << "[İHA-" << _id << "] Kalkış BAŞARISIZ: " << takeoff_result << '\n';
        return false;
    }
    std::cout << "[İHA-" << _id << "] Kalkış yapılıyor, irtifa bekleniyor...\n";
    
    std::this_thread::sleep_for(std::chrono::seconds(8));

    mavsdk::Offboard::PositionNedYaw initial_setpoint{};
    initial_setpoint.north_m = 0.0f;
    initial_setpoint.east_m = 0.0f;
    initial_setpoint.down_m = -2.5f; 
    initial_setpoint.yaw_deg = 0.0f;

    _offboard->set_position_ned(initial_setpoint);

    mavsdk::Offboard::Result offboard_result = _offboard->start();
    if (offboard_result != mavsdk::Offboard::Result::Success) {
        std::cerr << "[İHA-" << _id << "] Offboard moduna geçiş BAŞARISIZ: " << offboard_result << '\n';
        return false;
    }

    std::cout << "[İHA-" << _id << "] Offboard moduna BAŞARIYLA geçildi. Otonomi aktif.\n";
    return true;
}

void SwarmMember::update_target(Eigen::Vector3d target_ned, float yaw_deg) {
    mavsdk::Offboard::PositionNedYaw setpoint{};
    
    setpoint.north_m = target_ned.x();
    setpoint.east_m = target_ned.y();
    setpoint.down_m = target_ned.z();
    setpoint.yaw_deg = yaw_deg;

    _offboard->set_position_ned(setpoint);
}

void SwarmMember::land() {
    std::cout << "[İHA-" << _id << "] İniş komutu alındı!\n";
    
    _offboard->stop(); 
    
    mavsdk::Action::Result land_result = _action->land();
    if (land_result != mavsdk::Action::Result::Success) {
        std::cerr << "[İHA-" << _id << "] İniş BAŞARISIZ: " << land_result << '\n';
    } else {
        std::cout << "[İHA-" << _id << "] İniş başarılı.\n";
    }
}

mavsdk::Telemetry::Position SwarmMember::get_current_pos() {
    return _telemetry->position();
}
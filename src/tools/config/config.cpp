#include "ismpc_cpp/tools/config/config.h"

namespace ismpc {

void Config::init_params() {
    std::string path(CMAKE_SOURCE_DIR);
    path.append("/config/config.yaml");
    YAML::Node config = YAML::LoadFile(path);

    save_log = config["save_log"].as<bool>();
    delta = config["delta"].as<Scalar>();

    N = config["N"].as<int>();
    P = config["P"].as<int>();
    C = config["C"].as<int>();
    W = config["W"].as<int>();

    T_p = P * delta;
    T_c = C * delta;

    des_vel_x = config["des_vel_x"].as<Scalar>();
    des_vel_y = config["des_vel_y"].as<Scalar>();
    des_omega = config["des_omega"].as<Scalar>();

    tail_type = config["tail_type"].as<TailType>();
}

}  // namespace ismpc

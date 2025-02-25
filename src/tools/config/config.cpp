#include "ismpc_cpp/tools/config/config.h"

namespace ismpc {

void Config::init_params() {
    std::string path(CMAKE_SOURCE_DIR);
    path.append("/config/config.yaml");
    YAML::Node config = YAML::LoadFile(path);

    RED << 1.0, 0.0, 0.0;
    PURPLE << 0.6, 0.44, 0.86;
    GREEN << 0.0, 1.0, 0.0;
    save_log = config["save_log"].as<bool>();
    delta = config["delta"].as<Scalar>();
    fs_duration = config["fs_duration"].as<Scalar>();
    nl = config["nl"].as<int>();

    N = config["N"].as<int>();
    P = config["P"].as<int>();
    C = config["C"].as<int>();

    T_p = P * delta;
    T_c = C * delta;

    des_vel_x = config["des_vel_x"].as<Scalar>();
    des_vel_y = config["des_vel_y"].as<Scalar>();
    des_omega = config["des_omega"].as<Scalar>();

    tail_type = config["tail_type"].as<TailType>();
}

}  // namespace ismpc

#include "ismpc_cpp/tools/config/robot_config.h"

namespace ismpc {

void RobotConfig::init_params() {
    std::string path(CMAKE_SOURCE_DIR);
    path.append("/config/config.yaml");
    YAML::Node config = YAML::LoadFile(path);
    std::string robot_path(CMAKE_SOURCE_DIR);
    std::string robot_name = config["robot"].as<std::string>();
    std::ostringstream oss;
    oss << "/config/robots/" << robot_name << ".yaml";
    robot_path.append(oss.str());
    YAML::Node robot_config = YAML::LoadFile(robot_path);

    h = robot_config["h"].as<Scalar>();
    theta_max = robot_config["theta_max"].as<Scalar>();
    step_height = robot_config["step_height"].as<Scalar>();
    foot_com_height = robot_config["foot_com_height"].as<Scalar>();

    g = 9.81;
    eta = std::sqrt(g / h);

    l = robot_config["l"].as<Scalar>();
    dax = robot_config["dax"].as<Scalar>();
    day = robot_config["day"].as<Scalar>();
    beta = robot_config["beta"].as<Scalar>();

    dxz = robot_config["dxz"].as<Scalar>();
    dyz = robot_config["dyz"].as<Scalar>();
    zmp_vx_max = robot_config["zmp_vx_max"].as<Scalar>();
    zmp_vy_max = robot_config["zmp_vy_max"].as<Scalar>();

    T_bar = robot_config["T_bar"].as<Scalar>();
    L_bar = robot_config["L_bar"].as<Scalar>();
    v_bar = L_bar / T_bar;
    alpha = robot_config["alpha"].as<Scalar>();
    ds_percentage = robot_config["ds_percentage"].as<Scalar>();
    ss_percentage = 1 - ds_percentage;

    left_foot_x = robot_config["left_foot_x"].as<Scalar>();
    left_foot_y = robot_config["left_foot_y"].as<Scalar>();
    right_foot_x = robot_config["right_foot_x"].as<Scalar>();
    right_foot_y = robot_config["right_foot_y"].as<Scalar>();
}

}  // namespace ismpc

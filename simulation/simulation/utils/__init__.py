import os
from dataclasses import dataclass
from typing import Type, TypeVar
import yaml
from omegaconf import OmegaConf

def project_root() -> str:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    max_iterations = 100  # Set a limit for the number of iterations
    for _ in range(max_iterations):
        if (
            "requirements.txt" in os.listdir(current_dir)
            or "setup.py" in os.listdir(current_dir)
            or "pyproject.toml" in os.listdir(current_dir)
        ):
            return current_dir
        current_dir = os.path.dirname(current_dir)
    raise FileNotFoundError(
        "requirements.txt not found in any parent directories within the iteration limit"
    )
    
def repo_root() -> str:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    max_iterations = 100  # Set a limit for the number of iterations
    for _ in range(max_iterations):
        if ".git" in os.listdir(current_dir):
            return current_dir
        current_dir = os.path.dirname(current_dir)
    raise FileNotFoundError(
        ".git not found in any parent directories within the iteration limit"
    )


T = TypeVar("T")


def load_config(file_path: str, config_class: Type[T]) -> T:
    """
    Load configuration from a YAML file and merge it into a configuration object of the specified class.

    Args:
      file_path (str): The path to the YAML configuration file.
      config_class (Type[T]): The class of the configuration object.

    Returns:
      T: The merged configuration object.
    """
    with open(file_path, "r") as file:
        try:
            config: T = OmegaConf.structured(config_class)
            data = OmegaConf.create(yaml.safe_load(file))
            OmegaConf.unsafe_merge(config, data)
            return config
        except yaml.YAMLError as e:
            print(f"Error decoding YAML: {e}")
            return config_class()


@dataclass
class Config:
    save_log: bool = False
    delta: float = 0.02
    fs_duration: float = 1.0
    nl: int = 1
    N: int = 500
    P: int = 1000
    C: int = 100
    robot: str = "hrp4"
    des_vel_x: float = 0.1
    des_vel_y: float = 0.0
    des_omega: float = 0.0
    tail_type: str = "PERIODIC"
    
@dataclass
class RobotConfig:
    # Geometric parameters
    h: float = 0.75
    l: float = 0.2
    theta_max: float = 0.3927  # np.pi/8
    step_height: float = 0.02  # [m]
    foot_com_height: float = 0.0  # [m]

    # Initial Position
    left_foot_x: float = 0.0
    left_foot_y: float = 0.1
    right_foot_x: float = 0.0
    right_foot_y: float = -0.1

    # Cruise parameters
    T_bar: float = 0.3
    L_bar: float = 0.3
    alpha: float = 0.5

    # Footsteps stuff
    dax: float = 0.3
    day: float = 0.05

    # ZMP stuff
    dxz: float = 0.2
    dyz: float = 0.05
    zmp_vx_max: float = 1.0
    zmp_vy_max: float = 1.0
    beta: float = 100.0

    # Timing stuff
    ds_percentage: float = 0.3

config = load_config(os.path.join(repo_root(), "config", "config.yaml"), Config)
robot_config = load_config(os.path.join(repo_root(), "config", "robots", f"{config.robot}.yaml"), RobotConfig)
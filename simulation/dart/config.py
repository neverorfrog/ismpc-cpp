from dataclasses import dataclass
from typing import Literal
import yaml

@dataclass
class Config:
    # Logging
    save_log: bool = False
    
    # Time parameters
    delta: float = 0.01
    first_fs_duration: float = 1.0
    
    # Duration parameters
    N: int = 500
    P: int = 1000
    C: int = 100
    W: int = 100
    
    # Robot configuration
    robot: str = "hrp4"
    
    # Desired velocities
    des_vel_x: float = 0.1
    des_vel_y: float = 0.0
    des_omega: float = 0.0
    
    # Tail configuration
    tail_type: Literal["PERIODIC", "TRUNCATED", "ANTICIPATIVE"] = "PERIODIC"
    
    @classmethod
    def from_yaml(cls, path: str) -> "Config":
        with open(path, 'r') as f:
            config_dict = yaml.safe_load(f)
        return cls(**config_dict)
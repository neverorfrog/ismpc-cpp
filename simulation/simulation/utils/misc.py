import os
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

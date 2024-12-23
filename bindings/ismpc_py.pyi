from typing import Annotated

from numpy.typing import ArrayLike

class EndEffector:
    def __init__(self) -> None: ...
    @property
    def pose(self) -> Pose3: ...
    @property
    def vel(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(3), order="C")]: ...
    @property
    def acc(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(3), order="C")]: ...
    def __str__(self) -> str: ...

class Footsteps:
    def __init__(self) -> None: ...
    @property
    def timestamps(self) -> list[float]: ...
    @property
    def num_predicted_footsteps(self) -> int: ...
    @property
    def num_controlled_footsteps(self) -> int: ...
    @property
    def theta(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(None), order="C")]: ...
    @property
    def x(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(None), order="C")]: ...
    @property
    def y(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(None), order="C")]: ...
    @property
    def zmp_midpoints(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(None, None), order="F")]: ...

class FrameInfo:
    def __init__(self) -> None: ...
    @property
    def tk(self) -> float: ...
    @property
    def k(self) -> int: ...

class Pose2:
    def __init__(self) -> None: ...
    def rotation(self) -> float: ...
    @property
    def translation(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(2), order="C")]: ...

class Pose3:
    def __init__(self) -> None: ...
    @property
    def rotation(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(3, 3), order="F")]: ...
    @property
    def translation(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(3), order="C")]: ...

class Reference:
    def __init__(self) -> None: ...
    def get_velocity(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(3), order="C")]: ...
    def get_trajectory(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(None, None), order="F")]: ...

class State:
    def __init__(self) -> None: ...
    @property
    def com(self) -> EndEffector: ...
    @property
    def zmp_pos(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(3), order="C")]: ...
    @property
    def zmp_vel(
        self,
    ) -> Annotated[ArrayLike, dict(dtype="float64", shape=(3), order="C")]: ...
    @property
    def left_foot(self) -> EndEffector: ...
    @property
    def right_foot(self) -> EndEffector: ...
    def __str__(self) -> str: ...

class WalkEngine:
    def __init__(self) -> None: ...
    def update(self) -> None: ...
    def get_footsteps(self) -> Footsteps: ...
    def get_reference(self) -> Reference: ...
    def get_state(self) -> State: ...
    def get_walk_state(self) -> WalkState: ...
    def get_frame_info(self) -> FrameInfo: ...
    def set_reference_velocity(
        self, arg0: float, arg1: float, arg2: float, /
    ) -> None: ...
    def get_history(self) -> list[State]: ...
    def get_walk_history(self) -> list[WalkState]: ...
    def get_footstep_history(self) -> list[Pose2]: ...
    def get_timestamp_history(self) -> list[float]: ...

class WalkState:
    @property
    def current_footstep_timestamp(self) -> float: ...
    def __str__(self) -> str: ...

from typing import Dict, List


REDUNDANT_DOFS: Dict[str, List[str]] = {
    "hrp4": [
        "NECK_Y",
        "NECK_P",
        "R_SHOULDER_P",
        "R_SHOULDER_R",
        "R_SHOULDER_Y",
        "R_ELBOW_P",
        "L_SHOULDER_P",
        "L_SHOULDER_R",
        "L_SHOULDER_Y",
        "L_ELBOW_P",
    ],
    "nao": [
        "HeadYaw",
        "HeadPitch",
        "LWristYaw",
        "LHand",
        "RWristYaw",
        "RHand",
    ],
}

from bimanual import SingleArm
from typing import Dict, Any
import zmq
import json
import numpy as np

with open('calibration.json') as f:
    cal = json.load(f)
centers = cal['centers']
invert = cal['invert']
scale = 2 * np.pi / 4095

def test_follower(single_arm: SingleArm):
    s = zmq.Context().socket(zmq.PULL)
    s.bind("tcp://0.0.0.0:5000")
    
    while(1):
        data = json.loads(s.recv_string())
        pos = np.zeros(7)
        for i in range(1, 7):
            k = str(i)
            v = data[f'joint{i}']
            idx = i - 1 if i < 5 else i
            pos[idx] = (centers[k] - v if i in invert else v - centers[k]) * scale
        single_arm.set_joint_positions(pos)


if __name__ == "__main__":
    arm_config: Dict[str, Any] = {
        "can_port": "can1",
        "type": 0,
    }
    single_arm = SingleArm(arm_config)
    test_follower(single_arm)
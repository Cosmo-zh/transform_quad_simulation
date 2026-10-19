#!/usr/bin/env python3
"""Summarize actual folding windows from a transform_control SITL ULog."""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog

parser = argparse.ArgumentParser()
parser.add_argument("log", type=Path)
args = parser.parse_args()
ulog = ULog(str(args.log))
messages = ulog.logged_messages
starts = [m for m in messages if "state=folding" in m.message or "state=unfolding" in m.message]
results = []
for start in starts:
    ends = [m for m in messages if m.timestamp > start.timestamp and
            ("state=partial" in m.message or "state=open" in m.message)]
    if not ends:
        raise RuntimeError("Unfinished folding movement")
    end = ends[0]
    # Include three seconds of settling after each movement.
    def window(name):
        data = ulog.get_dataset(name).data
        mask = (data["timestamp"] >= start.timestamp) & (data["timestamp"] <= end.timestamp + 3_000_000)
        assert np.any(mask), name
        return {key: value[mask] for key, value in data.items()}

    att = window("vehicle_attitude")
    w, x, y, z = [att[f"q[{i}]"] for i in range(4)]
    roll = np.degrees(np.arctan2(2 * (w*x+y*z), 1-2*(x*x+y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    pos = window("vehicle_local_position")
    allocation = window("control_allocator_status")
    motors = window("actuator_motors")
    outputs = np.stack([motors[f"control[{i}]"] for i in range(4)])
    result = {
        "action": "fold" if "state=folding" in start.message else "open",
        "start_s": start.timestamp / 1e6,
        "movement_s": (end.timestamp-start.timestamp) / 1e6,
        "max_abs_roll_deg": float(np.max(np.abs(roll))),
        "max_abs_pitch_deg": float(np.max(np.abs(pitch))),
        "max_horizontal_displacement_m": float(np.max(np.hypot(pos["x"]-pos["x"][0], pos["y"]-pos["y"][0]))),
        "max_vertical_displacement_m": float(np.max(np.abs(pos["z"]-pos["z"][0]))),
        "motor_min": float(np.min(outputs)), "motor_max": float(np.max(outputs)),
        "torque_achieved_percent": float(100*np.mean(allocation["torque_setpoint_achieved"])),
        "max_unallocated_torque": float(max(np.max(np.abs(allocation[f"unallocated_torque[{i}]"])) for i in range(3))),
    }
    results.append(result)
land = ulog.get_dataset("vehicle_land_detected").data
summary = {"log": str(args.log), "movements": results,
           "final_landed": bool(land["landed"][-1]),
           "takeoff_detected": any("Takeoff detected" in m.message for m in messages),
           "dropout_count": len(ulog.dropouts)}
print(json.dumps(summary, indent=2))
assert len(results) == 6, "Expected three complete fold/open cycles"
assert summary["takeoff_detected"] and summary["final_landed"]
for result in results:
    assert result["max_abs_roll_deg"] < 5 and result["max_abs_pitch_deg"] < 5
    assert result["max_horizontal_displacement_m"] < .5
    assert result["max_vertical_displacement_m"] < .3
    assert result["motor_min"] > .01 and result["motor_max"] < .99
    assert result["torque_achieved_percent"] == 100

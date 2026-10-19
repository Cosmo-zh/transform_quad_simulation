# Motor 1 Forward Plus Frame SITL Validation

## Result

On 2026-10-11, PX4 SITL and Gazebo Harmonic completed a 3 m takeoff,
three 0 -> 0.30 -> 0 fold/open cycles, and a landing with airframe 5101.
The transition setting was 6 seconds per full stroke. Each measured
partial movement lasted 1.78 seconds between state messages.

The analysis window covers each movement and the following 3 seconds.

| Metric | Worst result across six movements |
| --- | --- |
| Absolute roll | 0.369 degrees |
| Absolute pitch | 0.259 degrees |
| Horizontal displacement from window start | 0.0659 m |
| Vertical displacement from window start | 0.0249 m |
| Motor command range | 0.5730 to 0.5801 |
| Torque setpoint achieved | 100% of samples |
| Maximum absolute unallocated torque component | 6.61e-8 |
| ULog dropouts | 0 |

Takeoff and final landed state were confirmed from the ULog.

## Evidence

- Console: `build/transform_plus_check_20261011_125845/console.txt`
- ULog: `build/transform_plus_check_20261011_125845/log/2026-10-11/04_59_14.ulg`
- Analysis: `Tools/analyze_transform_plus_log.py`

The console also contains MAVLink command-ack queue loss messages. The
module publishes actuator commands at 50 Hz; this run does not validate
reliable external MAVLink action acknowledgement. The flight movements
were requested through the PX4 console.

## Model and Scope

Airframe 5101 uses `transform_quad_plus`, with motor 1 front, motor 2 rear,
motor 3 left, and motor 4 right. The four arm joint origins and orientations
are rotated +45 degrees in Gazebo FLU relative to the unchanged base/IMU.
This corresponds to the -45 degree FRD transform used by the control geometry.
Meshes continue to reference the original `transform_quad` asset directory.

The original SDF and controller use slightly different hinge coordinates.
Forward kinematics of the SDF gives motor-center widths of 280.00 mm open,
271.73 mm at 30%, and 195.14 mm at 100%. Controller geometry predicts
280.01, 270.58, and 192.33 mm respectively. These are geometric calculations,
not measurements of actual joint tracking, and exclude propeller diameter.
The mismatch predates the frame rotation and should be reconciled with
mechanical measurements before interpreting the model as an exact replica.

Only 30% folding was flight-tested. Full folding, RC channel gating, real
PWM mapping, servo backlash, physical angle tracking, and actual aircraft
performance are outside this validation. No hardware firmware was uploaded.

## Reproduce

```sh
python3 Tools/simulation/gz/tools/generate_transform_plus.py
make px4_sitl_default
python3 Tools/transform_plus_sitl_check.py
```

The runner uses a fresh parameter directory and a private Gazebo transport
partition. Its no-RC autonomous-flight settings apply only to that run.

For interactive Gazebo simulation:

```sh
PX4_SYS_AUTOSTART=5101 make px4_sitl gz_transform_quad_plus
```

Check `transform_control status` for `geometry: plus (motor 1 front)`.
Keep the existing `limit 0.30` for reproducing these results.

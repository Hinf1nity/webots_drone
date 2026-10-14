# Cerro Rico SIL result

Scenario: low_light
Overall: FAIL

| Check | Result | Value | Limit |
| --- | --- | --- | --- |
| completed_duration | PASS | 130.0 | 129.9 |
| clock_monotonic | PASS | 0 | 0 |
| altitude | PASS | 0.010045230626072217 | 0.1 |
| attitude | PASS | 2.040320598556225 | 5.0 |
| tilt | PASS | 9.889158427650015 | 15.0 |
| contacts | PASS | 0 | 0 |
| motor_saturation | PASS | 0.0 | 0.01 |
| odometry | FAIL | 0.35609881938376925 | 0.3 |
| odometry_end_drift | FAIL | 0.6108321296577317 | 0.3 |
| route_progress | FAIL | 2.171143668525166 | 2.5 |
| route_return | FAIL | 1.3610918903042724 | 0.5 |
| map_updates | PASS | 40 | 20 |
| map_points | PASS | 46330 | 1000 |
| map_surface | PASS | 0.4145970213051936 | 0.5 |
| imu_rate | PASS | 76.44748327029386 | 20.0 |
| camera_rate | PASS | 116.23572623569201 | 5.0 |
| lidar_rate | PASS | 8.231351981351983 | 3.0 |
| odometry_rate | PASS | 8.245863401122532 | 3.0 |
| camera_exposure | FAIL | 8.468639322916667 | 10.0 |
| camera_contrast | FAIL | 3.519003033427024 | 8.0 |

Ground truth is scoring-only. The odometry-driven route controller is a test fixture, not a production autonomous navigation stack.
This run validates only the entrance corridor and stated conditions, not the full mine, real-world flight, ArduPilot SITL, or gas sensing.

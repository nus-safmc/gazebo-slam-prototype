# Gazebo Models

Simulation models used by the ToF SLAM system.

## Drone Sensor

### TOF-Ring

PX4-compatible ring of 8 VL53L7CX-equivalent depth cameras arranged at 45-degree
intervals around a central `tof_base_link`.  The ring is included as a nested model
inside the PX4 `x500_small_tof` airframe.

| Property        | Value             |
|-----------------|-------------------|
| Sensors         | 8 depth cameras   |
| Resolution      | 8 x 8 pixels      |
| Format          | `R_FLOAT32`       |
| HFOV            | 45 degrees        |
| Range           | 0.05 -- 4.0 m     |
| Update rate     | 15 Hz             |
| Mass            | 0.001 kg          |

Each sensor publishes on a model-scoped Gazebo topic of the form
`~/depth/tof_N` (N = 1..8) so that multiple vehicles can coexist.

### vl53l7cx (reference)

Legacy individual sensor model.  Retained for reference but no longer mounted on
the active drone platform.

## Environment

### inner_wall / perimeter_wall / pillar

Arena obstacle and boundary primitives used by the world files.

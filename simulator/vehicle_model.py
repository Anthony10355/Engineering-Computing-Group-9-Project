"""Ego vehicle: geometry and the kinematic bicycle model.

Copied unchanged (apart from imports) from the original research simulator.

Frame: world X north, Y east, yaw measured clockwise from north.  Positive
yaw rate and positive steering both mean "turning right".

The reference point is the REAR-AXLE centre.  Equations::

    X' = v cos(yaw),  Y' = v sin(yaw),  yaw' = v tan(delta) / L,  v' = a

Steering is rate- and angle-limited; acceleration follows the command through
a first-order lag (time constant 0.2 s).  No tires, no load transfer, no
reversing.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math


@dataclass(frozen=True)
class EgoVehicleGeometry:
    length_m: float = 4.66
    width_m: float = 1.89
    wheelbase_m: float = 3.00
    rear_axle_to_front_m: float = 3.90     # rear axle -> front bumper

    @property
    def rear_axle_to_rear_m(self) -> float:
        return self.length_m - self.rear_axle_to_front_m

    @property
    def center_offset_m(self) -> float:
        """Distance from the rear axle forward to the centre of the footprint."""
        return 0.5 * (self.rear_axle_to_front_m - self.rear_axle_to_rear_m)


EGO_GEOMETRY = EgoVehicleGeometry()


@dataclass(frozen=True)
class KinematicBicycleConfig:
    wheelbase_m: float = 3.0
    max_steering_rad: float = 0.55
    max_steering_rate_rps: float = 0.8
    max_acceleration_mps2: float = 3.0
    max_braking_mps2: float = 8.0
    acceleration_time_constant_s: float = 0.2   # 0 = ideal actuator


@dataclass(frozen=True)
class BicycleState:
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    acceleration_mps2: float = 0.0
    steering_rad: float = 0.0


class KinematicBicycle:
    def __init__(self, config: KinematicBicycleConfig | None = None) -> None:
        self.config = config or KinematicBicycleConfig()

    def yaw_rate_rps(self, state: BicycleState) -> float:
        return state.speed_mps * math.tan(state.steering_rad) / self.config.wheelbase_m

    def step(self, state: BicycleState, steering_command_rad: float,
             acceleration_command_mps2: float, dt_s: float) -> BicycleState:
        cfg = self.config
        if not (math.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError("dt_s must be finite and positive")
        if not (math.isfinite(steering_command_rad) and math.isfinite(acceleration_command_mps2)):
            raise ValueError("commands must be finite")

        steer_cmd = max(-cfg.max_steering_rad, min(cfg.max_steering_rad, steering_command_rad))
        max_delta = cfg.max_steering_rate_rps * dt_s
        steering = state.steering_rad + max(-max_delta, min(max_delta, steer_cmd - state.steering_rad))

        accel_cmd = max(-cfg.max_braking_mps2, min(cfg.max_acceleration_mps2, acceleration_command_mps2))
        tau = cfg.acceleration_time_constant_s
        accel = accel_cmd if tau == 0.0 else (
            state.acceleration_mps2 + (accel_cmd - state.acceleration_mps2) * (1.0 - math.exp(-dt_s / tau)))

        speed = max(0.0, state.speed_mps + accel * dt_s)
        realized_accel = (speed - state.speed_mps) / dt_s
        v_mid = 0.5 * (state.speed_mps + speed)
        yaw_rate = v_mid * math.tan(steering) / cfg.wheelbase_m
        yaw_mid = state.yaw_rad + 0.5 * yaw_rate * dt_s
        return replace(
            state,
            x_m=state.x_m + v_mid * math.cos(yaw_mid) * dt_s,
            y_m=state.y_m + v_mid * math.sin(yaw_mid) * dt_s,
            yaw_rad=state.yaw_rad + yaw_rate * dt_s,
            speed_mps=speed,
            acceleration_mps2=realized_accel,     # at standstill a brake demand gives 0 (no reversing)
            steering_rad=steering,
        )

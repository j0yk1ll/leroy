"""Entirely synthetic boot scan demonstration; never opens hardware/ROS."""
import math
from boot_localization_core import BootLocalizationModel, Observation


def synthetic(t, heading, **updates):
    data = dict(
        now=t, image_at=t, obstacle_at=t, yaw_at=t,
        localization_at=t, yaw_rad=math.radians(heading),
        localization_status="initializing",
        independent_stop_proven=True,
        verified_stop_distance=True,
        exclusive_command_authority=True,
        qualified_yaw_odometry=True,
        validated_tf_geometry=True,
        measured_swept_footprint=True,
        calibrated_obstacle_classifier=True,
        full_swept_envelope_observed=True,
        full_swept_envelope_clear=True,
        physical_motion_release=True,
    )
    data.update(updates)
    return Observation(**data)


def main():
    model = BootLocalizationModel()
    assert model.decision().state == "INHIBITED"
    model.request_start()
    reality = model.tick(Observation(now=0.0))
    assert reality.state == "FAULT_LATCHED"
    print("BOOT_UNQUALIFIED=" + reality.state + ":" + reality.reason)

    model = BootLocalizationModel()
    model.request_start()
    script = [
        (0.0, 0), (1.01, 0),
        (1.7, -90), (2.01, -90),
        (2.7, 90), (3.01, 90),
        (3.7, 0), (4.01, 0),
    ]
    for t, heading in script:
        d = model.tick(synthetic(t, heading))
        print("SIM_TICK: t={:.2f} yaw={} state={} target={}".format(
            t, heading, d.state,
            None if d.diagnostic_yaw_target_rad is None
            else round(math.degrees(d.diagnostic_yaw_target_rad))))
    assert model.state == "NEEDS_PARALLAX"
    assert model.decision().diagnostic_yaw_target_rad is None
    print("BOOT_SYNTHETIC_SWEEP=PASS_NEEDS_PARALLAX")

    model = BootLocalizationModel()
    model.request_start()
    model.tick(synthetic(0.0, 0))
    model.tick(synthetic(1.01, 0))
    stopped = model.tick(synthetic(
        1.20, 0, full_swept_envelope_clear=False))
    assert stopped.state == "FAULT_LATCHED"
    assert stopped.diagnostic_yaw_target_rad is None
    assert model.tick(synthetic(1.40, 0)).state == "FAULT_LATCHED"
    print("BOOT_SYNTHETIC_OBSTACLE=PASS_LATCHED")
    print("BOOT_MODEL=PASS_NO_ROS_NO_DEVICES_NO_ACTUATION")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""No-device, no-firmware-build test of the IMU mounting config and transform.

Compiles a tiny pure-C harness with the host compiler in a temporary
directory. NEVER calls make, Podman, serial, flashing, ROS or hardware.
Tests default robot-frame parity and alternate valid/invalid profiles.
"""
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
INCLUDE = ROOT / "include"
CONFIG = (INCLUDE / "imu_mount_config.h").read_text()
TRANSFORM = (INCLUDE / "imu_axis_transform.h").read_text()
IMU_SOURCE = (ROOT / "src/imu.c").read_text()

HARNESS = r"""
#include <stdint.h>
#include "imu_axis_transform.h"
int main(void) {
    const int32_t accel_raw[3] = { 100, -200, 300 };
    const int32_t gyro_bias_corrected[3] = { 11, -22, 33 };
    int32_t accel[3], gyro[3];
    imu_mount_transform_vector(accel_raw, accel);
    imu_mount_transform_vector(gyro_bias_corrected, gyro);
    if (accel[0] != EXP_AX || accel[1] != EXP_AY || accel[2] != EXP_AZ)
        return 11;
    if (gyro[0] != EXP_GX || gyro[1] != EXP_GY || gyro[2] != EXP_GZ)
        return 12;
    return 0;
}
"""


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError("BLOCKED_CONFIG_ANCHOR_CHANGED:" + old)
    return text.replace(old, new)


def compile_fixture(compiler, directory, config, expected, should_pass):
    root = directory / ("case_" + str(len(list(directory.iterdir()))))
    root.mkdir()
    (root / "imu_mount_config.h").write_text(config)
    (root / "imu_axis_transform.h").write_text(TRANSFORM)
    (root / "main.c").write_text(HARNESS)
    output = root / "fixture"
    args = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(root)]
    args.extend("-D" + name + "=" + str(value)
                for name, value in zip(
                    ("EXP_AX", "EXP_AY", "EXP_AZ", "EXP_GX", "EXP_GY", "EXP_GZ"),
                    expected))
    args += [str(root / "main.c"), "-o", str(output)]
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, check=False)
    if should_pass:
        if result.returncode:
            raise RuntimeError("BLOCKED_COMPILE_VALID_CONFIG:" + result.stderr[-1500:])
        invoked = subprocess.run([str(output)], check=False, timeout=5)
        if invoked.returncode:
            raise RuntimeError("BLOCKED_TRANSFORM_CHANGED:" + str(invoked.returncode))
    elif result.returncode == 0:
        raise RuntimeError("BLOCKED_INVALID_CONFIG_COMPILED")


def main():
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        raise RuntimeError("BLOCKED_HOST_C_COMPILER_NOT_FOUND")
    # Check the checked-in firmware really uses the config helper for
    # both acceleration and bias-corrected gyro, not just tests.
    if IMU_SOURCE.count("imu_mount_transform_vector(") != 2:
        raise RuntimeError("BLOCKED_FIRMWARE_NOT_USING_CONFIG_TWICE")
    if '#include "imu_axis_transform.h"' not in IMU_SOURCE:
        raise RuntimeError("BLOCKED_IMU_CONFIG_INCLUDE")
    if "const int32_t gyro_sensor[3]" not in IMU_SOURCE:
        raise RuntimeError("BLOCKED_GYRO_BIAS_BEFORE_ROTATION")
    if not all(re.search(r"imu_mount_transform_vector\("
                         + r"\s*" + re.escape(source), IMU_SOURCE)
               for source in ("accel_sensor", "gyro_sensor")):
        raise RuntimeError("BLOCKED_ACCEL_OR_GYRO_NOT_MAPPED")

    # We know the accepted MCU telemetry was robot-aligned (-X,-Y,+Z).
    expected_macros = {
        "IMU_MOUNT_ROBOT_X_SOURCE": "IMU_MOUNT_RAW_AXIS_X",
        "IMU_MOUNT_ROBOT_X_SIGN": "(-1)",
        "IMU_MOUNT_ROBOT_Y_SOURCE": "IMU_MOUNT_RAW_AXIS_Y",
        "IMU_MOUNT_ROBOT_Y_SIGN": "(-1)",
        "IMU_MOUNT_ROBOT_Z_SOURCE": "IMU_MOUNT_RAW_AXIS_Z",
        "IMU_MOUNT_ROBOT_Z_SIGN": "(+1)",
    }
    for k, v in expected_macros.items():
        if f"#define {k} {v}" not in CONFIG:
            raise RuntimeError("BLOCKED_DEFAULT_MOUNT_CHANGED:" + k)

    with tempfile.TemporaryDirectory(prefix="leroy-imu-config-") as temp:
        directory = Path(temp)
        compile_fixture(compiler, directory, CONFIG,
                        (-100, 200, 300, -11, 22, 33), True)
        print("IMU_MOUNT_DEFAULT_PARITY=PASS_NEG_X_NEG_Y_POS_Z")
        alt = CONFIG
        alt = replace_once(alt, "#define IMU_MOUNT_ROBOT_X_SOURCE IMU_MOUNT_RAW_AXIS_X",
                           "#define IMU_MOUNT_ROBOT_X_SOURCE IMU_MOUNT_RAW_AXIS_Y")
        alt = replace_once(alt, "#define IMU_MOUNT_ROBOT_Y_SOURCE IMU_MOUNT_RAW_AXIS_Y",
                           "#define IMU_MOUNT_ROBOT_Y_SOURCE IMU_MOUNT_RAW_AXIS_Z")
        alt = replace_once(alt, "#define IMU_MOUNT_ROBOT_Z_SOURCE IMU_MOUNT_RAW_AXIS_Z",
                           "#define IMU_MOUNT_ROBOT_Z_SOURCE IMU_MOUNT_RAW_AXIS_X")
        for name in ("X", "Y"):
            alt = replace_once(alt, f"#define IMU_MOUNT_ROBOT_{name}_SIGN (-1)",
                               f"#define IMU_MOUNT_ROBOT_{name}_SIGN (+1)")
        compile_fixture(compiler, directory, alt,
                        (-200, 300, 100, -22, 33, 11), True)
        print("IMU_MOUNT_ALTERNATE_RIGHT_HANDED_AXIS_PERMUTATION=PASS")
        reflection = replace_once(CONFIG,
                                  "#define IMU_MOUNT_ROBOT_Y_SIGN (-1)",
                                  "#define IMU_MOUNT_ROBOT_Y_SIGN (+1)")
        compile_fixture(compiler, directory, reflection, (0,) * 6, False)
        print("IMU_MOUNT_REFLECTION=REJECTED_AT_COMPILE_TIME")
        duplicate = replace_once(CONFIG,
                                 "#define IMU_MOUNT_ROBOT_Y_SOURCE IMU_MOUNT_RAW_AXIS_Y",
                                 "#define IMU_MOUNT_ROBOT_Y_SOURCE IMU_MOUNT_RAW_AXIS_X")
        compile_fixture(compiler, directory, duplicate, (0,) * 6, False)
        print("IMU_MOUNT_DUPLICATE_AXIS=REJECTED_AT_COMPILE_TIME")
    print("IMU_MOUNT_CONFIG=PASS_SOURCE_ONLY_NO_HARDWARE")
    print("IMU_MOUNT_FIRMWARE_FLASH=NOT_PERFORMED")
    print("IMU_MOUNT_MCU_SERIAL_AND_MOTOR_ACCESS=NONE")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print("IMU_MOUNT_CONFIG=BLOCKED_" + str(exc), file=sys.stderr)
        sys.exit(2)

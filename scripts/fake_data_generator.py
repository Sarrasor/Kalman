from typing import Optional, List

from pathlib import Path
import argparse
import json

import numpy as np

# TODO: Move to config

# Time step for ground truth data points
DT = 10000
# Sensor measurements time jitter
TIME_JITTER = 1000

# Lidar measurement interval
LIDAR_DT = 100000
# Lidar noise std
LIDAR_STD = [0.05, 0.05]

# Radar measurement interval
RADAR_DT = 100000
# Radar noise std
RADAR_STD = [0.3, 0.03, 0.3]


def generate_ground_truth(
    variant: Optional[str] = None, num_points: int = 1000
) -> List[dict]:
    if not variant:
        variant = "stationary"

    result = []
    if variant == "stationary":
        result = generate_stationary(num_points=num_points)
    elif variant == "linear":
        result = generate_linear(num_points=num_points)

    return result


def generate_stationary(num_points: int) -> List[dict]:
    result = []
    state = [1.0, 1.0, 0.0, 0.0]

    for i in range(num_points):
        result.append({"timestamp": i * DT, "state": state})

    return result


def generate_linear(num_points: int) -> List[dict]:
    result = []
    state = [-10.0, 1.0, 1.0, 1.0]

    for i in range(num_points):
        result.append({"timestamp": i * DT, "state": state.copy()})
        state[0] += state[2] * DT / 1e6
        state[1] += state[3] * DT / 1e6

    return result


def generate_lidar_measurement(state: dict) -> dict:
    mean = [0.0, 0.0]
    covariance = [
        [LIDAR_STD[0] ** 2, 0.0],
        [0.0, LIDAR_STD[1] ** 2],
    ]
    noise = np.random.multivariate_normal(mean, covariance, 1)

    return {
        "timestamp": state["timestamp"]
        + np.random.randint(-TIME_JITTER, TIME_JITTER),
        "sensor_type": 2,
        "data": (state["state"][:2] + noise).flatten().tolist(),
    }


def generate_radar_measurement(state: dict) -> dict:
    mean = [0.0, 0.0, 0.0]
    covariance = [
        [RADAR_STD[0] ** 2, 0.0, 0.0],
        [0.0, RADAR_STD[1] ** 2, 0.0],
        [0.0, 0.0, RADAR_STD[2] ** 2],
    ]
    noise = np.random.multivariate_normal(mean, covariance, 1)
    x = state["state"]

    rho = np.sqrt(x[0] ** 2 + x[1] ** 2)
    phi = np.arctan2(x[1], x[0])
    rho_dot = (x[0] * x[2] + x[1] * x[3]) / rho

    measurement = np.array([rho, phi, rho_dot])
    return {
        "timestamp": state["timestamp"]
        + np.random.randint(-TIME_JITTER, TIME_JITTER),
        "sensor_type": 3,
        "data": (measurement + noise).flatten().tolist(),
    }


def generate_data(gt_variant: str, object_init: bool) -> dict:
    result = {}
    ground_truth = generate_ground_truth(variant=gt_variant)
    result["ground_truth"] = ground_truth

    if object_init:
        # TODO: generate noisy initial state
        result["object"] = ground_truth[0]
        result["object"]["state_std"] = [0.01, 0.01, 0.01, 0.01]

    measurements = []
    prev_time_lidar = 0
    prev_time_radar = 0
    for gt in ground_truth:
        if gt["timestamp"] - prev_time_lidar >= LIDAR_DT:
            measurements.append(generate_lidar_measurement(gt))
            prev_time_lidar = gt["timestamp"]
        if gt["timestamp"] - prev_time_radar >= RADAR_DT:
            measurements.append(generate_radar_measurement(gt))
            prev_time_radar = gt["timestamp"]
    result["measurements"] = sorted(measurements, key=lambda m: m["timestamp"])
    return result


def main(output_folder: str, gt_variant: str, object_init: bool) -> None:
    output_folder = Path(output_folder)
    json_path = output_folder / "data.json"

    result = generate_data(gt_variant, object_init)

    with open(str(json_path), "w") as outfile:
        json.dump(result, outfile, indent=4, sort_keys=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Creates fake data")
    parser.add_argument("path", type=str, help="Path to the output folder")
    parser.add_argument(
        "variant",
        choices=["stationary", "linear"],
        type=str.lower,
        help="Variant of ground truth data",
    )
    parser.add_argument(
        "--init",
        action="store_true",
        help="Flag to generate object's initial state",
    )
    args = parser.parse_args()
    main(args.path, args.variant, args.init)

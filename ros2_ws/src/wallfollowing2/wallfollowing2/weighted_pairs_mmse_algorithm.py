import numpy as np


# numerical settings
epsilon = 1.0e-6  # small positive constant for numerical stability

# lidar preprocessing
downsample_resolution_deg = 1.0  # target angular resolution after averaging raw LiDAR samples
front_angle_limit_deg = 90.0  # only use LiDAR returns within +/- this front-facing angle
cone_width_deg = 5  # each cone spans five 1-degree samples
cone_half_width_deg = 2  # center sample +/- this many degrees gives five samples total
cone_step_deg = 5  # cone centers are spaced every five degrees
max_valid_distance_m = 30.0  # reject implausible or saturated ranges beyond this distance

# weighted-pairs tuning
k_J = 14.0  # converts cone model-fit cost into confidence
k_delta = 10.0  # penalizes large left/right distance disparity within a pair
angular_prior_exponent = 1.0  # exponent p used in sin(|alpha|)^p
min_pair_angle_deg = 10.0  # ignore pairs too close to straight ahead

# setpoint tuning
use_dynamic_lookahead = True  # when false, always use lookahead_distance_min
lookahead_distance_min = 0.3  # minimum forward setpoint distance when local geometry looks curved or asymmetric
lookahead_distance_max = 1.0  # maximum forward setpoint distance when local geometry looks straight and symmetric
k_lookahead = 20.0  # maps weighted pair disparity into a straightness score for dynamic lookahead
y_max = 0.35  # lateral setpoint clamp magnitude
min_turn_radius = 0.75  # minimum physically allowed signed turn-radius magnitude
straight_y_threshold_m = 0.02  # treat smaller lateral offsets as straight driving

# speed tuning
v_min = 0.75  # fallback and minimum commanded speed
v_max = 1.25  # maximum commanded speed
k_curvature = 0.6  # penalizes speed as curvature magnitude increases


def isValidDistance(distance_m):
    return (
        np.isfinite(distance_m)
        and distance_m > 0.0
        and distance_m < max_valid_distance_m
    )


def getScanRangesMeters(scan):
    if "ranges_m" in scan:
        return np.asarray(scan["ranges_m"], dtype=np.float64)

    if "x_local" in scan and "y_local" in scan:
        x_local = np.asarray(scan["x_local"], dtype=np.float64)
        y_local = np.asarray(scan["y_local"], dtype=np.float64)
        return np.hypot(x_local, y_local)

    return np.asarray(scan["ranges_px"], dtype=np.float64)


def preprocess_lidar(scan):
    angles_deg = np.asarray(scan["angles_deg"], dtype=np.float64)
    ranges_m = getScanRangesMeters(scan)
    hit_valid = np.asarray(scan["hit_valid"], dtype=bool)

    if len(angles_deg) == 0:
        return None

    averaged_angles_deg = []
    averaged_ranges_m = []
    averaged_valid = []

    degree_centers_deg = np.arange(
        -int(front_angle_limit_deg),
        int(front_angle_limit_deg) + 1,
        int(downsample_resolution_deg),
        dtype=np.int32,
    )

    for center_deg in degree_centers_deg:
        bin_mask = (
            (angles_deg >= center_deg - 0.5 * downsample_resolution_deg)
            & (angles_deg < center_deg + 0.5 * downsample_resolution_deg)
        )

        if not np.any(bin_mask):
            averaged_angles_deg.append(float(center_deg))
            averaged_ranges_m.append(np.nan)
            averaged_valid.append(False)
            continue

        bin_ranges = ranges_m[bin_mask]
        bin_valid = (
            hit_valid[bin_mask]
            & np.isfinite(bin_ranges)
            & (bin_ranges > 0.0)
            & (bin_ranges < max_valid_distance_m)
        )

        if np.count_nonzero(bin_valid) == 0:
            averaged_angles_deg.append(float(center_deg))
            averaged_ranges_m.append(np.nan)
            averaged_valid.append(False)
            continue

        averaged_angles_deg.append(float(center_deg))
        averaged_ranges_m.append(float(np.mean(bin_ranges[bin_valid])))
        averaged_valid.append(True)

    averaged_angles_deg = np.asarray(averaged_angles_deg, dtype=np.float64)
    averaged_ranges_m = np.asarray(averaged_ranges_m, dtype=np.float64)
    averaged_valid = np.asarray(averaged_valid, dtype=bool)

    angle_to_index = {
        int(angle_deg): idx
        for idx, angle_deg in enumerate(averaged_angles_deg.astype(np.int32))
    }

    return {
        "angles_deg": averaged_angles_deg,
        "angles_rad": np.deg2rad(averaged_angles_deg),
        "ranges_m": averaged_ranges_m,
        "valid": averaged_valid,
        "angle_to_index": angle_to_index,
    }


def compute_cone_cost(cone_distances_m):
    cone_distances_m = np.asarray(cone_distances_m, dtype=np.float64)

    if len(cone_distances_m) != cone_width_deg:
        return None
    if np.any(~np.isfinite(cone_distances_m)):
        return None
    if np.any(cone_distances_m <= 0.0):
        return None
    if np.any(cone_distances_m >= max_valid_distance_m):
        return None

    d_3 = float(cone_distances_m[cone_half_width_deg])
    if not isValidDistance(d_3):
        return None

    theta_offsets_deg = np.arange(-cone_half_width_deg, cone_half_width_deg + 1, dtype=np.float64)
    theta_offsets_rad = np.deg2rad(theta_offsets_deg)
    ideal_profile = d_3 / np.cos(theta_offsets_rad)
    residual = (cone_distances_m - ideal_profile) / (d_3 + epsilon)
    cost = float(np.mean(residual ** 2))
    confidence = float(np.exp(-k_J * cost))

    return {
        "cost": cost,
        "confidence": confidence,
        "center_distance_m": d_3,
    }


def extract_valid_cones(preprocessed_scan):
    valid_cones = {}

    max_center_angle_deg = (
        (int(front_angle_limit_deg) - cone_half_width_deg) // cone_step_deg
    ) * cone_step_deg
    candidate_centers_deg = np.arange(
        -max_center_angle_deg,
        max_center_angle_deg + 1,
        cone_step_deg,
        dtype=np.int32,
    )

    for center_deg in candidate_centers_deg:
        sample_angles_deg = np.arange(
            center_deg - cone_half_width_deg,
            center_deg + cone_half_width_deg + 1,
            dtype=np.int32,
        )

        indices = []
        for angle_deg in sample_angles_deg:
            idx = preprocessed_scan["angle_to_index"].get(int(angle_deg))
            if idx is None:
                indices = []
                break
            indices.append(idx)

        if len(indices) != cone_width_deg:
            continue

        if not np.all(preprocessed_scan["valid"][indices]):
            continue

        distances_m = preprocessed_scan["ranges_m"][indices]
        cone_result = compute_cone_cost(distances_m)
        if cone_result is None:
            continue

        center_angle_rad = np.deg2rad(float(center_deg))
        center_distance_m = cone_result["center_distance_m"]
        endpoint_local = np.array(
            [
                center_distance_m * np.cos(center_angle_rad),
                center_distance_m * np.sin(center_angle_rad),
            ],
            dtype=np.float64,
        )

        valid_cones[int(center_deg)] = {
            "center_angle_deg": float(center_deg),
            "center_angle_rad": center_angle_rad,
            "distances_m": distances_m,
            "cost": cone_result["cost"],
            "confidence": cone_result["confidence"],
            "center_distance_m": center_distance_m,
            "centerline_endpoint_local": endpoint_local,
        }

    return valid_cones


def compute_pair_value(left_cone, right_cone):
    alpha_rad = abs(float(left_cone["center_angle_rad"]))
    d_L = float(left_cone["center_distance_m"])
    d_R = float(right_cone["center_distance_m"])
    J_L = float(left_cone["cost"])
    J_R = float(right_cone["cost"])

    w_shape = float(np.exp(-k_J * (J_L + J_R)))
    disparity = (d_R - d_L) / (d_R + d_L + epsilon)
    w_disp = float(np.exp(-k_delta * (disparity ** 2)))
    w_alpha = float(np.sin(abs(alpha_rad)) ** angular_prior_exponent)
    value = float(w_shape * w_disp * w_alpha)
    lateral_estimate = float(0.5 * (d_R - d_L) * np.sin(abs(alpha_rad)))

    return {
        "alpha_rad": alpha_rad,
        "d_L": d_L,
        "d_R": d_R,
        "J_L": J_L,
        "J_R": J_R,
        "w_shape": w_shape,
        "w_disp": w_disp,
        "w_alpha": w_alpha,
        "value": value,
        "lateral_estimate": lateral_estimate,
    }


def compute_dynamic_lookahead(pair_results):
    if not use_dynamic_lookahead:
        return float(lookahead_distance_min), 0.0, 0.0

    if len(pair_results) == 0:
        return float(lookahead_distance_min), 0.0, 0.0

    pair_values = np.asarray([pair["value"] for pair in pair_results], dtype=np.float64)
    disparities = np.asarray(
        [
            (pair["d_R"] - pair["d_L"]) / (pair["d_R"] + pair["d_L"] + epsilon)
            for pair in pair_results
        ],
        dtype=np.float64,
    )
    value_sum = float(np.sum(pair_values))
    if value_sum <= epsilon:
        return float(lookahead_distance_min), 0.0, 0.0

    disparity_rms = float(np.sqrt(np.sum(pair_values * disparities ** 2) / (value_sum + epsilon)))
    straightness = float(np.exp(-k_lookahead * disparity_rms))
    lookahead_distance = float(
        lookahead_distance_min
        + (lookahead_distance_max - lookahead_distance_min) * straightness
    )
    return lookahead_distance, straightness, disparity_rms


def compute_weighted_setpoint(pair_results):
    lookahead_distance = float(lookahead_distance_min)
    if len(pair_results) == 0:
        return {
            "target_point_local": np.array([lookahead_distance_min, 0.0], dtype=np.float64),
            "y_star": 0.0,
            "y_sp": 0.0,
            "confidence": 0.0,
            "lookahead_distance": lookahead_distance,
            "straightness": 0.0,
            "disparity_rms": 0.0,
        }

    pair_values = np.asarray([pair["value"] for pair in pair_results], dtype=np.float64)
    lateral_estimates = np.asarray([pair["lateral_estimate"] for pair in pair_results], dtype=np.float64)
    angular_priors = np.asarray([pair["w_alpha"] for pair in pair_results], dtype=np.float64)

    value_sum = float(np.sum(pair_values))
    if value_sum <= epsilon:
        return {
            "target_point_local": np.array([lookahead_distance_min, 0.0], dtype=np.float64),
            "y_star": 0.0,
            "y_sp": 0.0,
            "confidence": 0.0,
            "lookahead_distance": lookahead_distance,
            "straightness": 0.0,
            "disparity_rms": 0.0,
        }

    y_star = float(np.sum(pair_values * lateral_estimates) / (value_sum + epsilon))
    lookahead_distance, straightness, disparity_rms = compute_dynamic_lookahead(pair_results)
    y_sp = float(np.clip(-y_star, -y_max, y_max))
    confidence = float(
        np.clip(
            value_sum / (float(np.sum(angular_priors)) + epsilon),
            0.0,
            1.0,
        )
    )
    target_point_local = np.array([lookahead_distance, y_sp], dtype=np.float64)

    return {
        "target_point_local": target_point_local,
        "y_star": y_star,
        "y_sp": y_sp,
        "confidence": confidence,
        "lookahead_distance": lookahead_distance,
        "straightness": straightness,
        "disparity_rms": disparity_rms,
    }


def pure_pursuit_radius(setpoint_local):
    x_sp = float(setpoint_local[0])
    y_sp = float(setpoint_local[1])

    if abs(y_sp) < straight_y_threshold_m:
        return np.inf, 0.0

    curvature = 2.0 * y_sp / (x_sp ** 2 + y_sp ** 2 + epsilon)
    if abs(curvature) < epsilon:
        return np.inf, 0.0

    radius_cmd = 1.0 / curvature
    radius_cmd = float(np.sign(radius_cmd) * max(abs(radius_cmd), min_turn_radius))
    return radius_cmd, float(curvature)


def compute_speed(confidence, curvature):
    speed_cmd = v_min + (v_max - v_min) * confidence / (1.0 + k_curvature * abs(curvature))
    return float(np.clip(speed_cmd, v_min, v_max))


def makeFallbackResult(status, cone_center_lines_local=None):
    setpoint_local = np.array([lookahead_distance_min, 0.0], dtype=np.float64)

    if cone_center_lines_local is None:
        cone_center_lines_local = np.zeros((0, 2, 2), dtype=np.float64)

    return {
        "speed_mps": v_min,
        "steering_radius_m": np.inf,
        "status": status,
        "debug": {
            "path_local": None,
            "target_point_local": setpoint_local,
            "left_boundary_local": None,
            "right_boundary_local": None,
            "cone_center_lines_local": cone_center_lines_local,
            "pair_angles_deg": np.array([], dtype=np.float64),
            "pair_values": np.array([], dtype=np.float64),
            "confidence": 0.0,
            "y_star": 0.0,
            "curvature": 0.0,
            "lookahead_distance": float(lookahead_distance_min),
            "straightness": 0.0,
            "disparity_rms": 0.0,
        },
    }


def runAutonomousAlgorithm(scan):
    preprocessed_scan = preprocess_lidar(scan)
    if preprocessed_scan is None:
        return makeFallbackResult("weighted pairs fallback | no lidar data")

    valid_cones = extract_valid_cones(preprocessed_scan)
    if len(valid_cones) == 0:
        return makeFallbackResult("weighted pairs fallback | no valid cones")

    pair_results = []
    cone_center_lines_local = []

    for cone in valid_cones.values():
        if abs(cone["center_angle_deg"]) < min_pair_angle_deg:
            continue
        cone_center_lines_local.append(
            np.vstack(
                [
                    np.zeros(2, dtype=np.float64),
                    cone["centerline_endpoint_local"],
                ]
            )
        )

    max_pair_angle_deg = int(front_angle_limit_deg) - cone_half_width_deg
    for alpha_deg in range(cone_step_deg, max_pair_angle_deg + 1, cone_step_deg):
        if abs(alpha_deg) < min_pair_angle_deg:
            continue

        left_cone = valid_cones.get(int(alpha_deg))
        right_cone = valid_cones.get(int(-alpha_deg))
        if left_cone is None or right_cone is None:
            continue

        pair_result = compute_pair_value(left_cone, right_cone)
        if pair_result["value"] <= 0.0:
            continue

        pair_results.append(pair_result)

    cone_center_lines_local = np.asarray(cone_center_lines_local, dtype=np.float64)

    if len(pair_results) == 0:
        return makeFallbackResult(
            "weighted pairs fallback | insufficient valid pair data",
            cone_center_lines_local=cone_center_lines_local,
        )

    setpoint_result = compute_weighted_setpoint(pair_results)
    setpoint_local = setpoint_result["target_point_local"]
    radius_cmd, curvature = pure_pursuit_radius(setpoint_local)
    speed_cmd = compute_speed(setpoint_result["confidence"], curvature)

    return {
        "speed_mps": speed_cmd,
        "steering_radius_m": radius_cmd,
        "status": (
            f"weighted pairs ok | conf={setpoint_result['confidence']:.2f} | "
            f"LA={setpoint_result['lookahead_distance']:.2f} m | "
            f"y*={setpoint_result['y_star']:+.2f} m | "
            f"setpoint=({setpoint_local[0]:.2f}, {setpoint_local[1]:+.2f}) m"
        ),
        "debug": {
            "path_local": None,
            "target_point_local": setpoint_local,
            "left_boundary_local": None,
            "right_boundary_local": None,
            "cone_center_lines_local": cone_center_lines_local,
            "pair_angles_deg": np.asarray(
                [np.rad2deg(pair["alpha_rad"]) for pair in pair_results],
                dtype=np.float64,
            ),
            "pair_values": np.asarray(
                [pair["value"] for pair in pair_results],
                dtype=np.float64,
            ),
            "confidence": setpoint_result["confidence"],
            "y_star": setpoint_result["y_star"],
            "curvature": curvature,
            "lookahead_distance": setpoint_result["lookahead_distance"],
            "straightness": setpoint_result["straightness"],
            "disparity_rms": setpoint_result["disparity_rms"],
        },
    }

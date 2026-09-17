"""Safety validation for MPC configurations."""


def validate_config(config: dict) -> tuple[bool, str]:
    """Validate an MPC configuration for safety.

    Args:
        config: MPC configuration dictionary.

    Returns:
        (is_valid, reason) — reason is empty string if valid.
    """
    errors = []

    # Horizon must be positive integer
    horizon = config.get("horizon")
    if horizon is None:
        errors.append("Missing 'horizon'")
    elif not isinstance(horizon, int) or horizon <= 0:
        errors.append(f"horizon must be positive integer, got {horizon}")
    elif horizon > 30:
        errors.append(f"horizon={horizon} exceeds max allowed (30)")

    # dt must be positive
    dt = config.get("dt")
    if dt is not None and (not isinstance(dt, (int, float)) or dt <= 0):
        errors.append(f"dt must be positive, got {dt}")

    # Q must be non-negative
    Q = config.get("Q")
    if Q is not None:
        for i, q in enumerate(Q):
            if q < 0:
                errors.append(f"Q[{i}] = {q} must be non-negative")

    # R must be non-negative
    R = config.get("R")
    if R is not None:
        for i, r in enumerate(R):
            if r < 0:
                errors.append(f"R[{i}] = {r} must be non-negative")

    # Steering limit
    max_steer = config.get("max_steer")
    if max_steer is not None:
        if max_steer <= 0 or max_steer > 1.57:  # ~90 degrees
            errors.append(f"max_steer={max_steer} out of safe range (0, 1.57]")

    # Acceleration limit
    max_accel = config.get("max_accel")
    if max_accel is not None:
        if max_accel <= 0 or max_accel > 10.0:
            errors.append(f"max_accel={max_accel} out of safe range (0, 10.0]")

    # Speed limit
    max_speed = config.get("max_speed")
    if max_speed is not None:
        if max_speed <= 0 or max_speed > 10.0:
            errors.append(f"max_speed={max_speed} out of safe range (0, 10.0]")

    # Vehicle length
    vehicle_length = config.get("vehicle_length")
    if vehicle_length is not None:
        if vehicle_length <= 0 or vehicle_length > 5.0:
            errors.append(f"vehicle_length={vehicle_length} out of safe range (0, 5.0]")

    if errors:
        return False, "; ".join(errors)
    return True, ""


def get_safe_config_range() -> dict:
    """Return the safe parameter ranges for MPC configuration.

    Returns:
        dict mapping parameter names to (min, max) tuples.
    """
    return {
        "horizon": (1, 30),
        "dt": (0.01, 1.0),
        "Q": (0.0, 100.0),
        "R": (0.0, 100.0),
        "max_steer": (0.01, 1.57),
        "max_accel": (0.1, 10.0),
        "max_speed": (0.1, 10.0),
        "vehicle_length": (0.05, 5.0),
        "num_iterations": (1, 10),
    }

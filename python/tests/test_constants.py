"""Tests for physical constants."""

import math

from src.constants import (
    AIR_DENSITY_SEA_LEVEL_KG_M3,
    BUNKER_DEPTH_MM,
    DRIVER_LOFT_DEG,
    GOLF_BALL_DIAMETER_M,
    GOLF_BALL_DRAG_COEFFICIENT,
    GOLF_BALL_MASS_KG,
    GRAVITY_M_S2,
    GREEN_SPEED_STIMP,
    HUMIDITY_PERCENT,
    IRON_7_LOFT_DEG,
    PI,
    PRESSURE_HPA,
    PUTTER_LOFT_DEG,
    ROUGH_HEIGHT_MM,
    SPEED_OF_LIGHT_M_S,
    TEMPERATURE_C,
    E,
)


def test_mathematical_constants() -> None:
    """Test mathematical constants match expected values."""
    assert math.pi == PI
    assert math.e == E


def test_gravity_constant() -> None:
    """Test gravity constant value and units."""
    # Standard gravity per ISO 80000-3:2006
    expected_gravity = 9.80665  # [m/s²]
    assert expected_gravity == GRAVITY_M_S2


def test_speed_of_light_constant() -> None:
    """Test speed of light constant value and units."""
    # Exact by definition, SI
    expected_speed = 299792458  # [m/s]
    assert expected_speed == SPEED_OF_LIGHT_M_S


def test_air_density_constant() -> None:
    """Test air density constant value and units."""
    # ISA at sea level, 15°C
    expected_density = 1.225  # [kg/m³]
    assert expected_density == AIR_DENSITY_SEA_LEVEL_KG_M3


def test_golf_ball_constants() -> None:
    """Test golf ball physical constants."""
    # USGA Rule 5-1 (1.620 oz max)
    expected_mass = 0.04593  # [kg]
    assert expected_mass == GOLF_BALL_MASS_KG

    # USGA Rule 5-2 (1.680 in min)
    expected_diameter = 0.04267  # [m]
    assert expected_diameter == GOLF_BALL_DIAMETER_M

    # Smooth ball at Re~150,000 per Bearman & Harvey 1976
    expected_drag = 0.25
    assert expected_drag == GOLF_BALL_DRAG_COEFFICIENT


def test_club_specifications() -> None:
    """Test club specification constants."""
    expected_driver_loft = 10.5  # [deg] Typical driver loft angle
    assert expected_driver_loft == DRIVER_LOFT_DEG
    expected_iron_loft = 34.0  # [deg] Standard 7-iron loft
    assert expected_iron_loft == IRON_7_LOFT_DEG
    expected_putter_loft = 3.0   # [deg] Standard putter loft
    assert expected_putter_loft == PUTTER_LOFT_DEG


def test_course_conditions() -> None:
    """Test course condition constants."""
    expected_green_speed = 10.0  # [ft] Fast green speed
    assert expected_green_speed == GREEN_SPEED_STIMP
    expected_rough_height = 25.0    # [mm] Medium rough height
    assert expected_rough_height == ROUGH_HEIGHT_MM
    expected_bunker_depth = 100.0   # [mm] Standard bunker depth
    assert expected_bunker_depth == BUNKER_DEPTH_MM


def test_atmospheric_conditions() -> None:
    """Test atmospheric condition constants."""
    expected_temperature = 20.0      # [°C] Standard temperature
    assert expected_temperature == TEMPERATURE_C
    expected_pressure = 1013.25    # [hPa] Standard atmospheric pressure
    assert expected_pressure == PRESSURE_HPA
    expected_humidity = 50.0   # [%] Standard relative humidity
    assert expected_humidity == HUMIDITY_PERCENT


def test_positive_constants() -> None:
    """Test that physical constants are positive."""
    positive_constants = [
        GRAVITY_M_S2,
        SPEED_OF_LIGHT_M_S,
        AIR_DENSITY_SEA_LEVEL_KG_M3,
        GOLF_BALL_MASS_KG,
        GOLF_BALL_DIAMETER_M,
        GOLF_BALL_DRAG_COEFFICIENT,
        DRIVER_LOFT_DEG,
        IRON_7_LOFT_DEG,
        PUTTER_LOFT_DEG,
        GREEN_SPEED_STIMP,
        ROUGH_HEIGHT_MM,
        BUNKER_DEPTH_MM,
        TEMPERATURE_C,
        PRESSURE_HPA,
    ]
    for constant in positive_constants:
        assert constant > 0, f"Constant {constant} should be positive"


def test_angle_ranges() -> None:
    """Test that angle constants are in reasonable ranges."""
    # Loft angles should be between 0 and 90 degrees
    max_angle = 90
    assert 0 < DRIVER_LOFT_DEG < max_angle
    assert 0 < IRON_7_LOFT_DEG < max_angle
    assert 0 < PUTTER_LOFT_DEG < max_angle


def test_temperature_range() -> None:
    """Test that temperature constants are in reasonable ranges."""
    # Temperature should be between -50 and 100°C for golf conditions
    min_temp = -50
    max_temp = 100
    assert min_temp < TEMPERATURE_C < max_temp


def test_humidity_range() -> None:
    """Test that humidity constants are in reasonable ranges."""
    # Humidity should be between 0 and 100%
    max_humidity = 100
    assert 0 <= HUMIDITY_PERCENT <= max_humidity

"""
Unit Tests for Visual Feature Tracker, Environmental Degradation, and Visual Odometry.
"""

import math

import numpy as np

from Localization.core.visual_feature_tracker import (
    VisualFeatureTracker,
)


def test_visual_tracker_clear_daylight():
    """Verify visual feature tracking produces accurate, low-covariance odometry in clear daylight."""
    tracker = VisualFeatureTracker(max_features=50)
    true_pose = np.array([15.0, -8.0, math.pi / 6])
    prior = true_pose.copy()

    meas = tracker.simulate_track_features(
        true_pose=true_pose,
        prior_estimate=prior,
        timestamp=100.0,
        sun_glare_factor=0.0,
        spray_occlusion_factor=0.0,
    )

    assert meas.is_valid
    assert meas.tracked_features >= 30
    assert meas.inlier_ratio > 0.80
    assert math.isclose(meas.x, 15.0, abs_tol=0.2)
    assert math.isclose(meas.y, -8.0, abs_tol=0.2)
    assert meas.covariance[0, 0] < 0.05  # Low position variance


def test_visual_tracker_sun_glare_degradation():
    """Verify direct sun glare degrades inlier ratio and inflates covariance."""
    tracker = VisualFeatureTracker()
    true_pose = np.array([0.0, 0.0, 0.0])

    # 70% sun glare blindness
    meas_glare = tracker.simulate_track_features(
        true_pose=true_pose,
        prior_estimate=true_pose,
        timestamp=100.0,
        sun_glare_factor=0.70,
    )

    # Covariance should be significantly higher due to dropped features
    assert meas_glare.tracked_features < 25
    assert meas_glare.covariance[0, 0] > 0.001


def test_visual_tracker_total_blindness_failure():
    """Verify complete visual occlusion triggers invalid status and fallback covariance."""
    tracker = VisualFeatureTracker(min_inlier_features=8)
    true_pose = np.array([10.0, 5.0, 0.0])
    prior_guess = np.array([10.2, 4.9, 0.0])

    # 100% spray/smoke occlusion
    meas_blind = tracker.simulate_track_features(
        true_pose=true_pose,
        prior_estimate=prior_guess,
        timestamp=100.0,
        spray_occlusion_factor=1.0,
    )

    assert not meas_blind.is_valid
    assert meas_blind.tracked_features < 8
    # Returns prior estimate with huge uncertainty
    assert math.isclose(meas_blind.x, 10.2)
    assert meas_blind.covariance[0, 0] >= 20.0

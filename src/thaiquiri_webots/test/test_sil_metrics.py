"""Regression tests for SIL scoring: preserve scale and reject unmatched data."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from thaiquiri_webots.sil_metrics import summarize_samples, surface_error, trajectory_error


def make_path():
    times = np.linspace(0, 10, 101)
    truth = np.column_stack([times, times*.3, np.sin(times)*.2, times*0+1,
                             np.zeros((len(times), 3)), np.ones(len(times))])
    return truth


def test_alignment_removes_origin_and_heading_only():
    truth = make_path()
    rotation = Rotation.from_euler('z', .8)
    translation = np.array([4, -3, 2])
    estimate = truth.copy()
    estimate[:,1:4] = rotation.inv().apply(truth[:,1:4]-translation)
    estimate[:,4:8] = rotation.inv().as_quat()
    result, _ = trajectory_error(truth, estimate)
    assert result['position_rmse_m'] < 1e-12


def test_scale_error_is_not_fitted_away():
    truth = make_path()
    estimate = truth.copy()
    estimate[:,1:3] *= 1.2
    result, _ = trajectory_error(truth, estimate)
    assert result['position_rmse_m'] > .3
    assert result['end_drift_m'] > .5


def test_missing_and_unmatched_data_fail_closed():
    assert trajectory_error([], [])[0]['available'] is False
    truth = make_path()
    estimate = truth.copy()
    estimate[:,0] += 30
    assert trajectory_error(truth, estimate)[0]['available'] is False
    assert summarize_samples([np.nan])['rmse'] is None
    assert summarize_samples([-2, 2])['rmse'] == pytest.approx(2)


def test_empty_map_is_unavailable_not_zero_error():
    assert surface_error([], [[0, 0, 0]]) == {'available': False}
    assert surface_error([[np.nan, 0, 0]], [[0, 0, 0]]) == {'available': False}


def test_surface_error_keeps_metric_distance():
    reference = [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    assert surface_error([[0, 0, .2], [1, 0, .2]], reference)['p95_m'] == pytest.approx(.2)

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from harness.metrics import EvalConfig, evaluate  # noqa: E402
from harness.tests.test_metrics_synthetic import build  # noqa: E402
from report import aggregate  # noqa: E402
from run_own_eval import jsonable  # noqa: E402


@pytest.fixture(scope='module')
def r():
    gt, pred = build()
    # through JSON, exactly as run_own_eval.py stores it (NaN -> null)
    return json.loads(json.dumps(jsonable(evaluate(gt, pred, EvalConfig()).to_dict())))


def test_aggregate_single_is_identity(r):
    a = aggregate([r])
    for k in ('mota', 'motp', 'idf1'):
        assert a[k] == pytest.approx(r[k])
    assert a['state_recall']['dynamic'] == pytest.approx(r['state_recall']['dynamic'])
    assert a['latency_mean']['to_dynamic'] == pytest.approx(r['latency_mean']['to_dynamic'])


def test_aggregate_sums_counts(r):
    a = aggregate([r, r])
    assert a['fp'] == 2 * r['fp'] and a['idsw'] == 2 * r['idsw']
    assert a['mota'] == pytest.approx(r['mota'])
    assert a['latency_censored']['to_dynamic'] == '0/2'

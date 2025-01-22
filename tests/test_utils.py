import pytest
from vortex_utils.python_utils import *

def test_ssa():
    assert ssa(0) == 0
    assert ssa(2 * np.pi) == 0
    assert ssa(3.5) == pytest.approx(-2.78, rel=0.01)
    assert ssa(-3.5) == pytest.approx(2.78, rel=0.01)

def test_euler_to_quat():
    quat = euler_to_quat(0, 0, 0)
    assert quat == pytest.approx([0, 0, 0, 1], rel=0.01)
    quat = euler_to_quat(1, 0, 0)
    assert quat == pytest.approx([0.479, 0, 0, 0.877], rel=0.01)
    quat = euler_to_quat(0, 1, 0)
    assert quat == pytest.approx([0, 0.479, 0, 0.877], rel=0.01)
    quat = euler_to_quat(0, 0, 1)
    assert quat == pytest.approx([0, 0, 0.479, 0.877], rel=0.01)
    quat = euler_to_quat(1, 1, 1)
    assert quat == pytest.approx([0.5709, 0.167, 0.5709, 0.565], rel=0.01)


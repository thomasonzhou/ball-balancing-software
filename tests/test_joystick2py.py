import joystick2py
import pytest

def test_shape():
    res = joystick2py.read_joystick()
    assert len(res) == 2
    assert len(res[0]) == 2
    (x, y), theta_rad = res
    assert 0.0<=x<=1.0 and 0.0<=y<=1.0
    assert isinstance(x, float)
    assert isinstance(y, float)
    assert isinstance(theta_rad, float)
    


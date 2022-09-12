import pytest

def foo(a,b):
    zk = ""
    if a >= 0:
        zk += " " * a
    zk += b
    return zk

@pytest.mark.parametrize("a", [-2,0,3,5])
@pytest.mark.parametrize("b", ["A", "BC"])                         
def test_foo(a,b):
    if a < 0:
        assert foo(a,b)==b
    if a >= 0:
        assert foo(a,b)==" "*a + b

import pytest

def foo(a,b):
    zk = ""
    if a >= 0:
        zk += " " * a
    zk += b
    return zk

@pytest.mark.parametrize("a,b,erg",
                         [ (5,"ABC","     ABC"),
                           (0,"ABC","ABC"),
                           (1,"ABC"," ABC"),
                           (-3,"ABC","ABC"),
                         ]
                        )
def test_foo(a,b,erg):
    assert foo(a,b)==erg

def foo(x,y):
    return x+y

def test1():
    assert foo(2,3)==5
    assert foo("ABC","DEF")=="ABCDEF"
    assert foo([1,2],[3,4])==[1,2,3,4]    

def test2():
    pass

def test3():
    return True

def test4():
    return False

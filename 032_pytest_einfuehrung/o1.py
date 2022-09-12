def foo(a,b,c):
    print("Ausgabe in foo()")
    return a+2*b+3*c

def test_foo1():
    print("Ausgabe in test_foo1()")
    assert foo(1,2,3)==14
    
def test_foo2():
    print("Ausgabe in test_foo2()")
    assert foo(2,2,2)==0

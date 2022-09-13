from datetime import datetime

def foo(x):    
    y = 1/x
    return y
    
    
def test_foo():        
    now = datetime.now()
    some_value = now.second % 2
        
    assert foo(some_value)==1
    
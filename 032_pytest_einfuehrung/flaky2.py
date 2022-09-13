from datetime import datetime
import time

def foo(x):    
    y = 1/x
    return y
    
    
def test_foo():    
    time.sleep(0.2)
    now = datetime.now()
    some_value = now.second % 2
    
    assert foo(some_value)==1
    
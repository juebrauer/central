def addiere(x,y):
    return x+y

def check1_addiere():
    assert addiere(2,2) == 4    
    
def check2_addiere():
    assert addiere(-2,-2) == -4
    
def test3_addiere():
    assert addiere(-17,30) == 13

def test4_addiere():
    assert addiere(-1,+1) == 0

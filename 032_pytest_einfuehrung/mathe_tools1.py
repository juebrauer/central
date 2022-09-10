def addiere(x,y):
    return x+y

def test1_addiere():
    assert addiere(2,2)==4
    
def test2_addiere():
    assert addiere(-2,-2)==-4
    
def test3_addiere():
    assert addiere(-17,30)==13

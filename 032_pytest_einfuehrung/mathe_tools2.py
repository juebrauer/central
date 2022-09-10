def addiere(x,y):
    return x+y

def test1_addiere():
    assert addiere(2,2) == 4
    
def test2_addiere():
    assert addiere(-2,-2) == -4
    
def test3_addiere():
    assert addiere(-17,30) == 13
    
def test4_addiere():
    assert addiere([1,2,3],[4,5,6]) == [5,7,9]
    
def testaddiere():
    assert addiere(-100,+100) == 0


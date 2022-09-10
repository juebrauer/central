def addiere(x,y):
    return x+y

def test1_addiere():
    #assert addiere(2,2) == 4
    pass
    
def addiere_test2():
    assert addiere(-2,-2) == -4
    
def addieretest3undauchnocheinpraefix():
    assert addiere(-17,30) == 13

def diehiernicht():
    assert addiere(-1,+1) == 0

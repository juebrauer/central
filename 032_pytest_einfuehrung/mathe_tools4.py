def addiere(x,y):
    return x+y

def check1_addiere():
    assert addiere(2,2) == 4    
    
def addiere_check2_xyz():
    assert addiere(-2,-2) == -4
    
def check3_addiere():
    assert addiere(-17,30) == 13

def und_noch_eine():
    assert addiere(-1,+1) == 0

import pytest

def foo(x,y):
    return x / y

def test1():
    assert foo(1,2)==0.5
    
def test2():
    with pytest.raises(ZeroDivisionError):
        foo(1,0)
    
class GehaltUnplausibel(Exception):
    
    def __init__(self, gehalt):
        self.error_msg = f"Ein Gehalt in Höhe von {gehalt} ist unplausibel!"
        
def gehalt_ueberpruefen(gehalt):
    if gehalt>=1000000:
        raise GehaltUnplausibel(gehalt)
        
def test4():
    with pytest.raises(GehaltUnplausibel) as exc_info:
        gehalt_ueberpruefen(2000000)
    assert exc_info.type is GehaltUnplausibel
    assert exc_info.value.error_msg == "Ein Gehalt in Höhe von 2000000 ist unplausibel!"


    


def ist_palindrom(s):
    return s == s[::-1]


def test_ist_palindrom1():
    assert ist_palindrom("")
    
def test_ist_palindrom2():
    assert ist_palindrom("a")
    
def test_ist_palindrom3():
    assert not ist_palindrom("abc")

def test_ist_palindrom4():
    assert ist_palindrom("aba")
    
def test_ist_palindrom5():
    assert ist_palindrom("CDC")

def test_ist_palindrom6():
    assert not ist_palindrom("A BA")



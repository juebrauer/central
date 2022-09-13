def f1(x,y):    
    return x+y

def f2(x,y):
    # Jetzt berechnen wir das Produkt
    if x<y:
        prod = x*y
    else:
        prod = x*(y**2)
    return prod

def f3(x,y):
    return x**2 + y**2

def test_f1():
    assert f1(2,9) == 11


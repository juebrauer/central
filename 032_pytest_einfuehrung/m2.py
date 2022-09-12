import pytest
import sys

def hole_tabelle_fuer_kunden_aus_datenbank(kunde):
    return "tabelle"

def test1():
    assert hole_tabelle_fuer_kunden_aus_datenbank("maier")=="tabelle"


@pytest.mark.skip(reason="Test geht noch nicht, kein Datenbankzugang")
def test2():
    pass
    
    
@pytest.mark.skipif(sys.platform == "linux", reason="Test geht nicht unter Linux")
def test3():
    pass


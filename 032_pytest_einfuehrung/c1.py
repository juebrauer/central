class kunde:
    
    def __init__(self, name, vorname, nr):
        self.name = name
        self.vorname = vorname
        self.nr = nr
        
    def normalisiere(self):
        self.vorname = " ".join(self.vorname.split())
        self.name = self.name.strip()
        
    def kapitalisiere(self):
        self.name = self.name.capitalize()
        self.vorname = " ".join([vorname.capitalize() for vorname in self.vorname.split()])

        
class Test_kunde:
    
    def test_kunde_anlegen(self): 
        k = kunde("  brauer ", " julius henry ", 938)
        assert k.name == "  brauer "
        assert k.vorname == " julius henry "
        assert k.nr == 938

    def test_kundenname_normalisieren(self):
        k = kunde("  brauer ", " julius henry ", 938)
        k.normalisiere()
        assert k.name == "brauer"
        assert k.vorname == "julius henry"
        assert k.nr == 938

    def test_kundenname_normalisieren_und_kapitalisieren(self):
        k = kunde("  brauer ", " julius henry ", 938)
        k.normalisiere()
        k.kapitalisiere()
        assert k.name == "Brauer"
        assert k.vorname == "Julius Henry"
        assert k.nr == 938              
        
        
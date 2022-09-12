import pytest

class bestellung:
    
    def __init__(self, name, vorname, liste : list):
        self.name = name
        self.vorname = vorname
        self.liste = liste
        
    def alles_klein(self):
        self.vorname = self.vorname.lower()
        self.name = self.name.lower()
        for i in range(0,len(self.liste)):            
            self.liste[i] = self.liste[i].lower()
            
    def alles_gross(self):
        self.vorname = self.vorname.upper()
        self.name = self.name.upper()
        for i in range(0,len(self.liste)):            
            self.liste[i] = self.liste[i].upper()
       
    
@pytest.mark.parametrize("n,v,l",
                         [
                             ("brauer", "jürgen", ["Pizza Hawai", "Chefsalat", "Tiramisu"]),
                             ("simpson", "homer", ["Lasagne", "XXL Familienpizza"])
                         ]
                        )
class Test_bestellung:
    
    def test_alles_klein(self, n,v,l):
        
        # Arrange
        b = bestellung(n,v,l)
        
        # Act
        b.alles_klein()
        
        # Assert
        assert b.name == n.lower()
        assert b.vorname == v.lower()
        assert b.liste == [e.lower() for e in b.liste]
        
        
    def test_alles_gross(self, n,v,l):
        
        # Arrange
        b = bestellung(n,v,l)
        
        # Act
        b.alles_gross()
        
        # Assert
        assert b.name == n.upper()
        assert b.vorname == v.upper()
        assert b.liste == [e.upper() for e in b.liste]
   
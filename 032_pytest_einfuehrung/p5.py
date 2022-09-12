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

    
class Test_bestellung:
    
    def test_alles_klein(self):
        
        # Arrange
        b = bestellung("brauer", "jürgen", ["Pizza Hawai", "Chefsalat", "Tiramisu"])
        
        # Act
        b.alles_klein()
        
        # Assert
        assert b.name == "brauer"
        assert b.vorname == "jürgen"
        assert b.liste == ["pizza hawai", "chefsalat", "tiramisu"]
        
        
    def test_alles_gross(self):
        
        # Arrange
        b = bestellung("brauer", "jürgen", ["Pizza Hawai", "Chefsalat", "Tiramisu"])
        
        # Act
        b.alles_gross()
        
        # Assert
        assert b.name == "BRAUER"
        assert b.vorname == "JÜRGEN"
        assert b.liste == ["PIZZA HAWAI", "CHEFSALAT", "TIRAMISU"]

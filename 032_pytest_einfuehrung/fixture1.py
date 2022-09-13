def foo(fname):
    text_file = open(fname, "r")
    data = text_file.read()
    text_file.close()
    return data


def test_foo(tmp_path):
    
    # Wir brauchen erstmal eine Textdatei!
    # Aber wo? Dazu nehmen wir das Testverzeichnis
    print("tmp_path = ", tmp_path)
    
    fname = "eine_datei.txt"
    text_file = open(fname, "w")
    text_file.write("ABC\n")
    text_file.write("123\n")
    text_file.close()
    
    assert foo(fname) == "ABC\n123\n"    
    
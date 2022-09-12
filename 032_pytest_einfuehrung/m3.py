import pytest
import re

def transform(zk):
    """    
    Remove all characters but letters
    and make all letters large    
    """
    regex = re.compile('[^a-zA-Z ]')
    zk_neu = regex.sub('', zk)
    zk_neu = zk_neu.upper()
    return zk_neu

@pytest.mark.change_input
@pytest.mark.input_lowercase_only
def test1():
    """
    Testing with small letters only
    """
    assert transform("abc")=="ABC"
    

@pytest.mark.change_input
@pytest.mark.input_lowercase_and_uppercase
def test2():
    """
    Testing with small and large letters
    """
    assert transform("aBc")=="ABC"
    
    
@pytest.mark.change_input
@pytest.mark.input_lowercase_only
def test3():
    """
    Testing with letters and whitespaces
    """
    assert transform("abc def")=="ABC DEF"

    
@pytest.mark.change_input
@pytest.mark.input_lowercase_only
def test4():
    """
    Testing with letters and whitespaces and numbers
    """
    assert transform("a1b2c3456 d789e000f1111")=="ABC DEF"
    
    
@pytest.mark.change_input
@pytest.mark.input_no_letters
def test5():
    """
    Testing wether only invalid characters string is
    transformed to an empty string
    """
    assert transform("1234567890")==""
    
    
@pytest.mark.do_not_change_input
@pytest.mark.input_uppercase_only
def test6():
    """
    Testing wether string that is already ok is kept as it is
    """
    assert transform("ABC DEF")=="ABC DEF"
    
    
@pytest.mark.do_not_change_input
@pytest.mark.input_no_letters
def test7():
    """
    Testing wether whitespace only character is kept as it is.
    """
    assert transform("  ")=="  "

    
@pytest.mark.do_not_change_input
@pytest.mark.input_uppercase_only
def test8():
    """
    Testing transformation of large valid string
    """
    zk = "ABC XYZ OPKLJKN JKDJ AFJKHJV KOKD" * 20
    assert transform(zk)==zk

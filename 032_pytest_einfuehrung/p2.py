import pytest

def ist_palindrom(s):
    return s == s[::-1]


@pytest.mark.parametrize("pali, erg",
                        [
                            ("", True),
                            ("a", True),
                            ("abc", False),
                            ("aba", True),
                            ("CDC", True),
                            ("A BA", False)
                        ]
                        )
def test_ist_palindrom(pali, erg):
    assert ist_palindrom( pali ) == erg

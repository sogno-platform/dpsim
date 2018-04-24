import dpsim.components.dp as dp
import pytest

def test_read():
    c = dp.Capacitor('c1', 1, 2, 1.234);

    assert c.capacitance == 1.234
    assert c.name == 'c1'
    assert c.node1 == 1
    assert c.node2 == 2

def test_write():
    c = dp.Capacitor('c1', 1, 1, 1.234);

    c.capacitance = 5

    assert c.capacitance == 5

def test_invalid():
    with pytest.raises(AttributeError) as e_info:
        c = dp.Capacitor('c1', 1, 1, 1.234);

        # dp.Capacitor does not have an attribute named 'doesnotexist'
        # Accessing it should throw a AttributeError exception!
        x = c.doesnotexist

def test_access():
    with pytest.raises(AttributeError) as e_info:
        c = dp.Capacitor('c1', 1, 1, 1.234);

        # Current is a read-only property.
        # This should throw a AttributeError exception!
        c.current = 5

def test_type():
    with pytest.raises(TypeError) as e_info:
        c = dp.Capacitor('c1', 1, 1, 1.234);

        # Capacitance is a real valued property.
        # Assigning a complex number should throw a TypeError exception!
        c.capacitance = 1j

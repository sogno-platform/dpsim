# SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
# SPDX-License-Identifier: MPL-2.0

# Attribute access on components: reading, writing, and the errors raised for
# names and types that do not exist.

import pytest

import dpsimpy


def capacitor(capacitance=1.234):
    c = dpsimpy.dp.ph1.Capacitor("c1")
    c.set_parameters(capacitance)
    return c


def test_read_attribute():
    c = capacitor()
    assert c.attr("C").get() == 1.234
    assert c.name() == "c1"


def test_read_property():
    c = capacitor()
    assert c.C == 1.234


def test_write_attribute():
    c = capacitor()
    c.attr("C").set(5.0)
    assert c.attr("C").get() == 5.0


def test_write_property():
    c = capacitor()
    c.C = 5.0
    assert c.C == 5.0


def test_unknown_attribute_name_raises():
    c = capacitor()
    with pytest.raises(Exception):
        c.attr("doesnotexist")


def test_unknown_python_member_raises():
    c = capacitor()
    with pytest.raises(AttributeError):
        c.doesnotexist


def test_wrong_attribute_type_raises():
    c = capacitor()
    # Capacitance is real valued; a complex number must be rejected.
    with pytest.raises(TypeError):
        c.attr("C").set(1j)

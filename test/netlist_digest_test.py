#!/usr/bin/env python3
"""Grades soc/netlist_digest.py's canonical form directly: a renumbered bufNormalize
cell is forgiven, and anything that changes a buffer's content or count is not.

Usage: netlist_digest_test.py [repo-root]     # defaults to this script's parent
"""

import copy
import importlib.util
import os
import sys
import unittest

ROOT = os.path.abspath(sys.argv.pop(1)) if len(sys.argv) > 1 else \
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = importlib.util.spec_from_file_location(
    "netlist_digest", os.path.join(ROOT, "soc", "netlist_digest.py"))
nd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nd)

PREFIX = "$auto$rtlil_bufnorm.cc:460:bufNormalize$"


def buf(width, a, y):
    return {"hide_name": 1, "type": "$buf", "parameters": {"WIDTH": width},
            "attributes": {}, "port_directions": {"A": "input", "Y": "output"},
            "connections": {"A": a, "Y": y}}


def design(numbers=(10, 11, 12)):
    cells = {
        f"{PREFIX}{numbers[0]}": buf("1", ["z"], [7]),
        f"{PREFIX}{numbers[1]}": buf("1", ["z"], [8]),
        f"{PREFIX}{numbers[2]}": buf("32", ["z"] * 32, list(range(20, 52))),
        "lut.1": {"type": "SB_LUT4", "parameters": {}, "attributes": {},
                  "connections": {"I0": [2], "O": [3]}},
    }
    return {"creator": "test", "modules": {"top": {
        "attributes": {"top": "1"}, "ports": {}, "cells": cells, "netnames": {}}}}


def cells(d):
    return d["modules"]["top"]["cells"]


class Canonical(unittest.TestCase):
    def test_renumbering_is_forgiven(self):
        self.assertEqual(nd.digest(design()), nd.digest(design((105, 106, 107))))

    def test_order_of_the_numbers_is_forgiven(self):
        self.assertEqual(nd.digest(design()), nd.digest(design((12, 11, 10))))

    def test_changed_parameter_is_not(self):
        other = design()
        cells(other)[f"{PREFIX}12"]["parameters"]["WIDTH"] = "31"
        self.assertNotEqual(nd.digest(design()), nd.digest(other))

    def test_changed_connection_is_not(self):
        other = design()
        cells(other)[f"{PREFIX}10"]["connections"]["Y"] = [9]
        self.assertNotEqual(nd.digest(design()), nd.digest(other))

    def test_dropped_buffer_is_not(self):
        other = design()
        del cells(other)[f"{PREFIX}11"]
        self.assertNotEqual(nd.digest(design()), nd.digest(other))

    def test_identical_buffers_are_counted(self):
        one = design()
        two = copy.deepcopy(one)
        cells(two)[f"{PREFIX}99"] = copy.deepcopy(cells(two)[f"{PREFIX}10"])
        self.assertNotEqual(nd.digest(one), nd.digest(two))

    def test_other_cell_names_are_kept(self):
        other = design()
        cells(other)["lut.2"] = cells(other).pop("lut.1")
        self.assertNotEqual(nd.digest(design()), nd.digest(other))

    def test_other_auto_names_are_kept(self):
        one, other = design(), design()
        for d, n in ((one, 5), (other, 6)):
            cells(d)[f"$auto$opt_clean.cc:1:x${n}"] = cells(d)["lut.1"]
        self.assertNotEqual(nd.digest(one), nd.digest(other))

    def test_input_is_not_mutated(self):
        d = design()
        before = copy.deepcopy(d)
        nd.canonical(d)
        self.assertEqual(d, before)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])

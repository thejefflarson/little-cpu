#!/usr/bin/env python3
"""Reads a mode-cover sby log into the per-goal sets the cover graders compare.

sby names each cover statement by the source range it was written at, so a site is
`<file>:<line>.<col>-<line>.<col>`. Three graders read logs this way --
nano/formal/complete-cover-probe.py, formal/memcheck-cover-probe.py and
formal/cover-depth-tie.py -- and they share this parse so a change in sby's wording
breaks all three the same way rather than one of them silently.
"""

import re


def parse(log_text, sv_name):
    """{"reached": sites, "unreached": sites, "steps": {site: first step reached}} for
    every cover statement in `sv_name` the log mentions. Summary lines and per-step
    lines both feed the two sets; only the per-step line carries a step."""
    name = re.escape(sv_name)
    site_re = re.compile(
        r"(?P<un>[Uu]n)?[Rr]eached cover statement.*?" + name + r":(?P<site>[\d.]+-[\d.]+)"
    )
    step_re = re.compile(
        r"Reached cover statement in step (?P<step>\d+).*?" + name + r":(?P<site>[\d.]+-[\d.]+)"
    )
    sites = {"reached": set(), "unreached": set(), "steps": {}}
    for m in site_re.finditer(log_text):
        sites["unreached" if m.group("un") else "reached"].add(m.group("site"))
    for m in step_re.finditer(log_text):
        sites["steps"].setdefault(m.group("site"), int(m.group("step")))
    return sites


def step_gap(sites):
    """None when every reached site has a first-reached step, else why not. A reached
    site with no step is a wording change in sby's per-step line, and a depth tie that
    reads no steps would otherwise pass having compared nothing."""
    missing = sorted(sites["reached"] - set(sites["steps"]))
    extra = sorted(set(sites["steps"]) - sites["reached"])
    if not missing and not extra:
        return None
    return (
        f"the log's reached sites and its per-step lines disagree (reached with no step: "
        f"{missing or 'none'}; a step with no reached site: {extra or 'none'}). sby's "
        "wording has probably moved, so no step can be graded against a depth."
    )

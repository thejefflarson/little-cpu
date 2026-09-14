#!/usr/bin/env python3
"""Fails a mode-cover job whose goals are first reached only at or after the depth of
the mode-bmc job it backs.

Usage: cover-depth-tie.py <cover-logfile> <sv-name> <bmc-sby>

formal/complete_cover.sby searches to depth 100 against complete.sby's 50, so a goal
first reached at step 50 or later would pass the anti-vacuity control while `complete`
never examines that retire. This reads the cover job's own log after it runs rather than
searching a second time. It is only sound for a harness whose assertion and cover goals
are both combinational, so the step a goal is reached is the step the assertion reads
it; formal/complete.sv's are. nano/formal/complete-cover-probe.py makes the same tie
inside its own probe, over the same parse.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cover_log
import depth_rules


def stop(message):
    """Exit 2: the inputs cannot be read, which is not a red verdict."""
    print(f"error: {message}", file=sys.stderr)
    return 2


def main():
    if len(sys.argv) != 4:
        print(f"usage: {sys.argv[0]} <cover-logfile> <sv-name> <bmc-sby>", file=sys.stderr)
        return 2
    log_path, sv_name, sby_path = sys.argv[1:]
    for path in (log_path, sby_path):
        if not os.path.isfile(path):
            return stop(f"{path} does not exist, so there is nothing to tie.")
    try:
        depth = depth_rules.read_sby_depth(sby_path)
    except ValueError as err:
        return stop(str(err))
    if depth is None:
        return stop(f"{sby_path} declares no `depth NNN` line in [options].")

    with open(log_path) as f:
        sites = cover_log.parse(f.read(), sv_name)
    if not sites["reached"]:
        return stop(f"{log_path} names no reached cover statement in {sv_name}, so there "
                    "is no step to tie.")
    gap = cover_log.step_gap(sites)
    if gap:
        return stop(gap)

    late = {site: step for site, step in sites["steps"].items() if step >= depth}
    if late:
        print(
            f"error: {sorted(late)} are first reached only at or after step {depth}, "
            f"{sby_path}'s own depth. The bmc job never examines a retire that late, so\n"
            "this anti-vacuity evidence outruns the check it is meant to back.",
            file=sys.stderr,
        )
        return 1
    print(f"{log_path}: {len(sites['steps'])} goals, the latest first reached at step "
          f"{max(sites['steps'].values())}, under {sby_path}'s depth {depth}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

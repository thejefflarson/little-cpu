#!/usr/bin/env python3
"""Grades soc/entropy_assess.py against sources whose answer is known, in both directions,
so a green assessment of a board's capture means the estimators could have said otherwise.

Usage: entropy_assess_test.py [repo-root]     # defaults to this script's parent
"""

import importlib.util
import os
import random
import sys


def load(root):
    path = os.path.join(root, "soc", "entropy_assess.py")
    spec = importlib.util.spec_from_file_location("entropy_assess", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check(root):
    ea = load(root)
    rng = random.Random(1)
    errors = []

    def expect(what, ok):
        if not ok:
            errors.append(what)

    expect("sample_bit is the parity of the low two bits of interval - 1",
           [ea.sample_bit(v) for v in (1, 2, 3, 4)] == [0, 1, 1, 0])

    uniform = [rng.randrange(3152, 3168) for _ in range(4096)]
    samples = [ea.sample_bit(v) for v in uniform]
    expect("a wide uniform interval spread must estimate near one bit per sample",
           min(ea.mcv_min_entropy(samples), ea.markov_min_entropy(samples)) > 0.9)
    expect("a wide uniform spread must not look periodic", abs(ea.best_period(samples)[1] - 0.5) < 0.05)

    constant = [3154] * 4096
    constant_samples = [ea.sample_bit(v) for v in constant]
    expect("a constant interval must estimate zero entropy", ea.mcv_min_entropy(constant_samples) < 0.01)
    expect("a constant interval must trip the proportion test and starve the corrector",
           ea.trng_model(constant)[3] == 8 and ea.trng_model(constant)[5] == 64)

    beat = [3154 + (0, 0, 2, 0, 2, 0, 2)[i % 7] for i in range(4096)]
    beat_samples = [ea.sample_bit(v) for v in beat]
    lag, match = ea.best_period(beat_samples)
    expect("a beat that repeats every seven intervals must show a self-match far from one half",
           lag % 7 == 0 and match > 0.99)
    expect("a beat must estimate far under the 0.5 bit the cutoff assumes",
           min(ea.mcv_min_entropy(beat_samples), ea.markov_min_entropy(beat_samples)) < 0.5)

    narrow = [3154 + rng.choice((0, 1)) for _ in range(4096)]
    narrow_samples = [ea.sample_bit(v) for v in narrow]
    expect("two intervals alternating at random is biased and must estimate under half a bit",
           ea.mcv_min_entropy(narrow_samples) < 0.5 or ea.markov_min_entropy(narrow_samples) < 0.5)

    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1
    print("entropy-assess: estimators agree with five sources of known answer.")
    return 0


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.exit(check(root))

"""The workflow-step splitter the workflow-grading tests share."""

import re


def jobs(text):
    """The workflow's jobs as {name: text}, split on the two-space-indented job keys."""
    lines = text.splitlines()
    start = next(i for i, l in enumerate(lines) if re.match(r"^jobs:\s*$", l))
    found, name = {}, None
    for line in lines[start + 1:]:
        m = re.match(r"^  ([A-Za-z0-9_-]+):\s*$", line)
        if m:
            name = m.group(1)
            found[name] = []
        elif name is not None:
            found[name].append(line)
    return {n: "\n".join(b) for n, b in found.items()}


def steps(text):
    """The workflow's steps as text chunks, split on the '- ' item marker."""
    lines = text.splitlines()
    start = next(i for i, l in enumerate(lines) if re.match(r"^\s*steps:\s*$", l))
    chunks, cur = [], None
    for line in lines[start + 1:]:
        if re.match(r"^\s{6}- ", line):
            if cur is not None:
                chunks.append(cur)
            cur = [line]
        elif cur is not None:
            cur.append(line)
    if cur is not None:
        chunks.append(cur)
    return ["\n".join(c) for c in chunks]

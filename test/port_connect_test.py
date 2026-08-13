#!/usr/bin/env python3
"""Asserts that every module instantiating `littlecpu` names every one of its ports.

Usage: port_connect_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. `rtl/littlecpu.v` gained `imem_fault` and
`soc/compare/bench_littlecpu.v` was not updated, so that input floated. yosys
warns and continues on an unconnected instance port, iverilog says nothing at
all, and the harness then placed 536 cells of a 6006-cell core -- 0.09x, which
icetime timed faster than the real design. The one grader that noticed is
`soc/compare/placed_vs_synth.py`, inside `make compare-timing`, which is a
measurement nobody runs per PR: it stayed red on main across several merges.
This check reports the cause instead of the symptom, at the moment the port is
added, and it is all fork and no work.

BOTH DIRECTIONS. A port on the module that an instantiation does not name is
red, and a connection naming a port the module no longer has is red too. The
second half is how a harness rots after a port is REMOVED, and it is the one
that is easy to forget: a frontend rejects it, but only on the flow that
elaborates that harness, which for the comparison harness is again the flow
nobody runs.

CONDITIONAL PORTS. Most of this module's port list sits inside `ifdef
RISCV_FORMAL`, with `RISCV_FORMAL_MEM_FAULT` and `RISCV_FORMAL_CSR_MCAUSE`
nested inside that; the formal harnesses define all three and the sim legs
define only the outer one, so no port set is owed by every site. Ports are
therefore graded by GUARD GROUP -- the tuple of macros a port is nested under:

  - The unguarded group is demanded of every site, and a connection to an
    unguarded port must itself be unguarded: connected only under a macro is
    floating without it, which is the defect this file is about.
  - A guarded group is all-or-nothing per site. A site naming NO port of the
    group is one that never defines the macro, and nothing is demanded of it; a
    site naming SOME of them must name them all. That is what keeps this check
    from being either useless or permanently red.
  - `RVFI_CONN` satisfies every group under RISCV_FORMAL. It expands from
    riscv-formal's generated rvfi_macros.vh, which is a build artifact and not
    in the tree, so no hermetic parse can see what it names. A file that
    declares RVFI wires is compiled with RISCV_FORMAL by definition, so it is
    required to carry `RVFI_CONN` -- otherwise dropping that one token would
    leave every rvfi wire undriven, and a check whose `rvfi_valid` never rises
    passes vacuously.

Every remaining omission is named in EXCEPTIONS below with the reason it stays,
and that table is graded both ways: an exception whose port is connected now, or
whose port no longer exists, is as red as an unconnected port.

Python rather than the shell the neighbouring checks use, because the parse has
to match parentheses and follow `ifdef nesting, and neither is something sed can
see. Hermetic: the standard library and `git ls-files`. No toolchain, no
simulator, no yosys, so this runs inside `make test` anywhere.
"""

import collections
import re
import subprocess
import sys
from pathlib import Path

MODULE = "littlecpu"
MODULE_FILE = "rtl/littlecpu.v"

# The macro whose group `RVFI_CONN` stands in for.
RVFI_MACRO = "RISCV_FORMAL"

# Macros that only a file compiled with RISCV_FORMAL can use. A file naming one
# of them has to connect the rvfi ports rather than being read as a site that
# never defines the macro.
RVFI_WIRE_MACROS = ("RVFI_WIRES", "RVFI_OUTPUTS")


# A declared port. `guard` is the tuple of macros it is nested under, empty for
# a port every build has.
Port = collections.namedtuple("Port", "name direction guard")

# One site's named omissions and the reason they stay. An entry that is only a
# path and a port teaches the next reader nothing and gets deleted by the next
# person tidying, so the reason is a field and it is printed.
Exemption = collections.namedtuple("Exemption", "path ports reason")

EXCEPTIONS = [
    Exemption(
        "test/testbench.v",
        (
            "rvfi_mode",
            "rvfi_ixl",
            "rvfi_csr_mcycle_rmask",
            "rvfi_csr_mcycle_wmask",
            "rvfi_csr_mcycle_rdata",
            "rvfi_csr_mcycle_wdata",
            "rvfi_csr_minstret_rmask",
            "rvfi_csr_minstret_wmask",
            "rvfi_csr_minstret_rdata",
            "rvfi_csr_minstret_wdata",
            "rvfi_csr_mscratch_rmask",
            "rvfi_csr_mscratch_wmask",
            "rvfi_csr_mscratch_rdata",
            "rvfi_csr_mscratch_wdata",
        ),
        # These are outputs, and an unnamed output drives nothing where an
        # unnamed INPUT floats. Connecting them anyway would be fourteen wires
        # no reader ever looks at.
        "the generated monitor both sim legs read has no CSR channel and no "
        "mode or ixl port, so these outputs have no reader in simulation",
    ),
]


def die(*lines):
    for line in lines:
        print(line, file=sys.stderr)
    raise SystemExit(1)


def strip_comments_and_strings(text):
    """Blank out comments and string bodies, keeping every newline in place.

    Line numbers have to survive: a diagnostic that cannot say where the
    connection is sends the reader looking through a 300-line port list.
    """
    out = []
    i = 0
    n = len(text)
    while i < n:
        two = text[i:i + 2]
        if two == "//":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
        elif two == "/*":
            while i < n and text[i:i + 2] != "*/":
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            out.append("  ")
            i += 2
        elif text[i] == '"':
            out.append(" ")
            i += 1
            while i < n and text[i] != '"':
                out.append("\n" if text[i] == "\n" else " ")
                i += 2 if text[i] == "\\" else 1
            out.append(" ")
            i += 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def matching_paren(text, start):
    """Offset of the `)` closing the `(` at `start`, or None."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    return None


DIRECTIVE = re.compile(r"^\s*`(ifdef|ifndef|elsif|else|endif)\b\s*(\w+)?")


def segments(where, text, first_line):
    """A parenthesised list's top-level comma-separated entries.

    Each is (body, lineno, guard), where `guard` is the tuple of macros active
    where the entry starts -- so a port and the connection to it are compared by
    the condition each is written under rather than by position.
    """
    guard = []
    body = ""
    body_line = first_line
    body_guard = ()
    depth = 0
    out = []
    lineno = first_line
    for line in text.split("\n"):
        d = DIRECTIVE.match(line)
        if d:
            kind, name = d.group(1), d.group(2)
            if kind == "ifdef":
                guard.append(name)
            elif kind == "ifndef":
                guard.append("!" + name)
            elif kind == "endif":
                if not guard:
                    die("error: %s has an unbalanced `endif at line %d."
                        % (where, lineno))
                guard.pop()
            else:
                die("error: %s has an `%s at line %d, and this check does not"
                    % (where, kind, lineno),
                    "model one. Nothing in this tree writes a port list that way,",
                    "so teach this script the shape rather than letting it read",
                    "one arm of the condition as if both were live.")
            lineno += 1
            continue
        for ch in line:
            if ch in "([{":
                depth += 1
            elif ch in ")]}":
                depth -= 1
            if ch == "," and depth == 0:
                out.append((body, body_line, body_guard))
                body = ""
                body_guard = ()
                body_line = lineno
                continue
            if not body.strip() and not ch.isspace():
                body_line = lineno
                body_guard = tuple(guard)
            body += ch
        body += "\n"
        lineno += 1
    out.append((body, body_line, body_guard))
    return out


PORT_DECL = re.compile(r"^\s*(input|output|inout)\b(.*)$", re.S)
CONNECTION = re.compile(r"^\s*\.\s*([A-Za-z_]\w*)\s*\((.*)\)\s*$", re.S)
BARE_MACRO = re.compile(r"^\s*`([A-Za-z_]\w*)\s*$", re.S)
RANGE = re.compile(r"\[[^\]]*\]")
WORD = re.compile(r"[A-Za-z_]\w*")


def stray_comma(where, lineno):
    die("error: %s has an empty entry at line %d -- a stray or trailing comma."
        % (where, lineno),
        "yosys accepts that spelling and iverilog rejects it, so it is a shape",
        "only the second frontend catches. Fix the list rather than teaching",
        "this check to skip empty entries.")


def module_ports(text):
    """The declared ports of `littlecpu`, in order."""
    m = re.search(r"\bmodule\s+%s\b" % MODULE, text)
    if not m:
        die("error: %s declares no `module %s`. This check reads that port list"
            % (MODULE_FILE, MODULE),
            "as the source of what every instantiation owes; if the module was",
            "renamed or moved, move this check with it.")
    open_paren = text.find("(", m.end())
    close = matching_paren(text, open_paren) if open_paren >= 0 else None
    if close is None:
        die("error: %s's port list has no closing parenthesis this check can"
            % MODULE_FILE,
            "find. It is parsed rather than pattern-matched, so an unreadable",
            "list stops the run instead of grading against an empty one.")
    first_line = text.count("\n", 0, open_paren) + 1
    where = MODULE_FILE + "'s port list"
    ports = []
    segs = segments(where, text[open_paren + 1:close], first_line)
    for body, lineno, guard in segs:
        if not body.strip():
            if len(segs) > 1:
                stray_comma(where, lineno)
            continue
        d = PORT_DECL.match(body)
        if not d:
            die("error: %s:%d is an entry in the port list that this check"
                % (MODULE_FILE, lineno),
                "cannot read as a port declaration:",
                "  %s" % " ".join(body.split()),
                "Every port is written with its own direction here for exactly",
                "that reason. Teach this script the new spelling rather than",
                "letting a port it cannot see go undemanded everywhere.")
        words = WORD.findall(RANGE.sub(" ", d.group(2)))
        if not words:
            die("error: %s:%d declares a direction and no port name."
                % (MODULE_FILE, lineno))
        ports.append(Port(words[-1], d.group(1), guard))
    if not ports:
        die("error: %s's port list parsed as empty, so every instantiation"
            % MODULE_FILE,
            "would owe nothing and this check would report green over any",
            "harness at all.")
    return ports


INSTANCE = re.compile(
    r"(?:^|[^A-Za-z0-9_.`])%s\s+([A-Za-z_]\w*)\s*(?:#\s*\([^()]*\)\s*)?\(" % MODULE,
    re.M,
)


def instantiations(path, text):
    """Every `littlecpu` instance in one file, as (instance, connections).

    A connection is (port-or-None, macro-or-None, lineno, expression, guard), so
    that a bare `RVFI_CONN` is carried alongside the named ones rather than
    dropped.
    """
    found = []
    for m in INSTANCE.finditer(text):
        open_paren = text.index("(", m.end() - 1)
        close = matching_paren(text, open_paren)
        if close is None:
            die("error: %s instantiates %s at line %d and the connection list"
                % (path, MODULE, text.count("\n", 0, m.start()) + 1),
                "has no closing parenthesis this check can find.")
        first_line = text.count("\n", 0, open_paren) + 1
        where = "%s's %s instance" % (path, MODULE)
        conns = []
        segs = segments(where, text[open_paren + 1:close], first_line)
        for body, lineno, guard in segs:
            if not body.strip():
                if len(segs) > 1:
                    stray_comma(where, lineno)
                continue
            c = CONNECTION.match(body)
            if c:
                conns.append((c.group(1), None, lineno, c.group(2).strip(), guard))
                continue
            b = BARE_MACRO.match(body)
            if b:
                conns.append((None, b.group(1), lineno, "", guard))
                continue
            die("error: %s:%d connects %s by something this check cannot read:"
                % (path, lineno, MODULE),
                "  %s" % " ".join(body.split()),
                "Only `.port(expr)` and a bare macro are graded. A connection by",
                "POSITION is the worst case of what this file exists to catch --",
                "it silently re-aims every port after the one that moved -- so it",
                "stops the run rather than being skipped.")
        found.append((m.group(1), conns))
    return found


def main():
    here = Path(__file__).resolve().parent
    repo = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else here.parent
    if not repo.is_dir():
        die("error: '%s' is not a directory, so there is nothing to check."
            % (sys.argv[1] if len(sys.argv) > 1 else repo))

    module_path = repo / MODULE_FILE
    if not module_path.is_file():
        die("error: %s is missing, so nothing says what ports %s has. If it"
            % (MODULE_FILE, MODULE),
            "moved, move this check with it.")

    ports = module_ports(strip_comments_and_strings(module_path.read_text()))
    by_name = dict((p.name, p) for p in ports)
    groups = {}
    for port in ports:
        groups.setdefault(port.guard, []).append(port.name)

    try:
        listed = subprocess.run(
            ["git", "-C", str(repo), "ls-files", "-z", "--", "*.v", "*.sv"],
            check=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout
    except (OSError, subprocess.CalledProcessError):
        listed = b""
    files = [f for f in listed.decode().split("\0") if f]
    if not files:
        die("error: cannot enumerate any tracked Verilog under %s. This check"
            % repo,
            "reads git's index, because what it guards is an instantiation",
            "arriving in a commit; a tree git cannot list is a scan of nothing",
            "reporting green.")

    rc = 0
    sites = 0
    exception_used = set()

    def excepted(path, port):
        for e in EXCEPTIONS:
            if e.path == path and port in e.ports:
                exception_used.add((path, port))
                return True
        return False

    def fail(*lines):
        for i, line in enumerate(lines):
            print(("error: " if i == 0 else "") + line, file=sys.stderr)
        print(file=sys.stderr)

    for path in sorted(files):
        if path == MODULE_FILE:
            continue
        text = strip_comments_and_strings((repo / path).read_text())
        for instance, conns in instantiations(path, text):
            sites += 1
            where = "%s's `%s` instance" % (path, instance)
            named = {}
            has_rvfi_conn = False
            for port, macro, lineno, expr, guard in conns:
                if macro == "RVFI_CONN":
                    has_rvfi_conn = True
                    continue
                if macro is not None:
                    die("error: %s:%d expands `%s in %s's connection list, and"
                        % (path, lineno, macro, MODULE),
                        "this check knows only `RVFI_CONN. A macro it cannot",
                        "expand would hide whatever it connects, so it stops here",
                        "rather than grading the ports it can still see.")
                if port not in by_name:
                    rc = 1
                    fail("%s:%d connects .%s, and %s has no such port."
                         % (path, lineno, port, MODULE),
                         "This is the half a one-way check would not have: the port",
                         "was removed or renamed in %s and this connection was" % MODULE_FILE,
                         "left behind. Delete it, or rename it to whatever the port",
                         "is called now.")
                    continue
                if not expr:
                    # Recorded as named anyway, so the group below reports this
                    # port once, in the words that fit it.
                    rc = 1
                    fail("%s:%d names .%s() with nothing in it, which for an"
                         % (path, lineno, port),
                         "input is the same floating pin as leaving the port out.",
                         "yosys warns and carries on, then folds away everything",
                         "behind it: that is how one harness placed a sixth of this",
                         "core and reported a critical path for it.")
                named[port] = guard
            if any(m in text for m in RVFI_WIRE_MACROS) and not has_rvfi_conn:
                rc = 1
                fail("%s declares RVFI wires, so it is compiled with %s, and"
                     % (path, RVFI_MACRO),
                     "%s carries no `RVFI_CONN. Every rvfi wire would be" % where,
                     "undriven, and a check whose rvfi_valid never rises passes",
                     "without looking at one instruction.")
            for guard, group in sorted(groups.items()):
                # A guarded group nobody here names is a macro this file never
                # defines, so its ports do not exist at this site and nothing is
                # owed. The unguarded group is owed by every site there is.
                covered = has_rvfi_conn and guard and guard[0] == RVFI_MACRO
                if guard and not covered and not any(p in named for p in group):
                    continue
                for port in group:
                    if port in named or covered or excepted(path, port):
                        continue
                    rc = 1
                    if guard:
                        fail("%s connects %s under %s but not .%s."
                             % (where, MODULE, " + ".join(guard), port),
                             "A guarded group is all or nothing here: naming some of",
                             "it says this file is built with that macro, and then",
                             "the rest of the group is floating rather than absent.",
                             "Connect it, or name it in EXCEPTIONS in",
                             "test/port_connect_test.py with the reason it stays.")
                    else:
                        fail("%s does not connect .%s, which %s has as an %s."
                             % (where, port, MODULE, by_name[port].direction),
                             "An unconnected input floats and yosys folds the whole",
                             "core away behind it -- this is exactly how",
                             "soc/compare/bench_littlecpu.v missed .imem_fault and",
                             "placed 536 cells of a 6006-cell core, which icetime",
                             "then timed faster than the real design. Add the",
                             "connection here, or tie it off with the reason.")

            for port, conn_guard in sorted(named.items()):
                if not conn_guard or by_name[port].guard:
                    continue
                rc = 1
                fail("%s connects .%s only under %s, and %s declares it"
                     % (where, port, " + ".join(conn_guard), MODULE),
                     "unconditionally. Without that macro the port is left",
                     "floating, which is the case this check exists for.")

    if not sites:
        die("error: no file in %s instantiates %s at all. This check grades" % (repo, MODULE),
            "instantiations against the port list, so finding none is a scan of",
            "nothing rather than a clean tree.")

    for e in EXCEPTIONS:
        for port in e.ports:
            if (e.path, port) in exception_used:
                continue
            rc = 1
            fail("EXCEPTIONS exempts .%s at %s, and that omission is gone --" % (port, e.path),
                 "the port is connected there now, or %s no longer has it, or" % MODULE,
                 "that file no longer instantiates %s. The entry says: %s." % (MODULE, e.reason),
                 "Delete it. An exemption kept past its reason is how the next one",
                 "gets waved through, and it is why this table is graded both ways.")

    if rc:
        raise SystemExit(1)

    exceptions = sum(len(e.ports) for e in EXCEPTIONS)
    print("%s: %d ports (%d unguarded) named at every one of %d instantiations, "
          "%d exceptions"
          % (MODULE, len(ports), len(groups.get((), [])), sites, exceptions))


if __name__ == "__main__":
    main()

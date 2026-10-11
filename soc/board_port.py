#!/usr/bin/env python3
"""Find the iCESugar-Pro's serial node by the debugger's USB VID/PID.

The loader is pinned to VID 0x1d50 / PID 0x602b; the reader has to be pinned to the same device, or a second
CDC device on the bus is read as the board. macOS names the node by enumeration order, so the VID/PID is the
only stable handle: this walks `ioreg`'s plist for every device with that pair and collects the callout nodes
beneath it; callers refuse zero and several.
"""

import plistlib
import subprocess
import sys

VID, PID = 0x1D50, 0x602B


def callout_nodes(tree, vid, pid, inside=False):
    """Callout nodes under every device matching vid/pid in an ioreg plist tree."""
    found = []
    nodes = tree if isinstance(tree, list) else [tree]
    for node in nodes:
        here = inside or (node.get("idVendor") == vid and node.get("idProduct") == pid)
        if here and isinstance(node.get("IOCalloutDevice"), str):
            found.append(node["IOCalloutDevice"])
        found += callout_nodes(node.get("IORegistryEntryChildren", []), vid, pid, here)
    return found


def ioreg_tree():
    out = subprocess.run(
        ["ioreg", "-r", "-c", "IOUSBHostDevice", "-l", "-a"], capture_output=True, check=True, timeout=20
    ).stdout
    return plistlib.loads(out) if out.strip() else []


def find_ports(vid=VID, pid=PID, tree=None):
    return sorted(set(callout_nodes(ioreg_tree() if tree is None else tree, vid, pid)))


def main():
    ports = find_ports()
    if len(ports) != 1:
        sys.exit("*** %d serial nodes for USB %04x:%04x: %s" % (len(ports), VID, PID, " ".join(ports)))
    print(ports[0])


if __name__ == "__main__":
    main()

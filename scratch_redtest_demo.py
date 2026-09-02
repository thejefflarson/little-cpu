"""Throwaway file to verify the Security Review workflow's new red
direction. Not part of any real feature; the branch this lives on is
deleted immediately after the check is confirmed to fail correctly."""

import subprocess


def run_backup(hostname):
    # Deliberately vulnerable: hostname is attacker-controlled and is
    # concatenated straight into a shell command.
    subprocess.run(f"ping -c 1 {hostname}", shell=True)

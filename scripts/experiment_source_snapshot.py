#!/usr/bin/env python3
"""Immutable pre-launch functional working-tree snapshot, including new untracked source files."""
import hashlib
import json
from pathlib import Path
import subprocess
import time


def freeze_sources(output, workspace):
    """Require a new snapshot directory; never rewrite a prior run's source provenance."""
    output, workspace = Path(output), Path(workspace)
    snapshot = output/'source_snapshot'
    snapshot.mkdir(exist_ok=False)
    command = ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z', '--',
               'src', 'scripts', 'docs/current', 'AGENTS.md', 'README.md']
    paths = subprocess.check_output(command, cwd=workspace).decode().strip('\0').split('\0')
    hashes = {}
    for name in sorted(set(paths)):
        source = workspace/name
        if not name or not source.is_file() or source.is_symlink():
            continue
        payload = source.read_bytes()
        target = snapshot/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
        hashes[name] = hashlib.sha256(payload).hexdigest()
    metadata = dict(frozen_stamp=time.time(), workspace=str(workspace.resolve()),
                    head=subprocess.check_output(['git', 'rev-parse', 'HEAD'],
                                                 cwd=workspace).decode().strip(), files=hashes)
    (output/'source_snapshot.json').write_text(json.dumps(metadata, indent=2))
    (output/'working_tree.patch').write_bytes(subprocess.check_output(
        ['git', 'diff', '--binary', '--', 'src', 'scripts', 'docs/current'], cwd=workspace))
    return metadata

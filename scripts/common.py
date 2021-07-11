#!/usr/bin/python3

import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent.resolve()
SOLUTIONS_DIR = REPO_DIR / 'solutions'
DEFAULT_SOLUTIONS_DIR = SOLUTIONS_DIR / 'submit'
DEFAULT_PROBLEMS_DIR = REPO_DIR / 'data' / 'problems'
if sys.platform == 'win32':
    EXE_DIR = REPO_DIR / 'vs' / 'x64' / 'Release'
else:
    EXE_DIR = REPO_DIR / 'src'

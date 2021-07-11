#!/usr/bin/python3

import argparse
import json
import shutil
import subprocess
import os
import re
import tqdm

from common import *

def main():
    parser = argparse.ArgumentParser(description='Brain Wall Visualizer')
    parser.add_argument('base_dir')
    args = parser.parse_args()

    file_list = []
    for root, dirs, files in os.walk(os.path.abspath(args.base_dir)):
        for f in files:
            if f.endswith('.json'):
                path = os.path.join(root, f)
                file_list.append(path)
    
    for path in file_list:
        mo = re.search(r'(\d+)', os.path.basename(path))
        if mo:
            num = mo.groups()[0]
            problem_path = os.path.join(DEFAULT_PROBLEMS_DIR, f'{num}.problem.json')

        subprocess.run([str(EXE_DIR / 'judge'), problem_path, path], cwd=EXE_DIR, stderr=subprocess.PIPE)
        with open(path, 'r') as fi:
            j = json.load(fi)

        result = j['meta']['judge']
        print(path, result['is_valid'], result['dislikes'] if result['is_valid'] else 'N/A',
            'Bonus' if result['gained_bonuses'] else '')

if __name__ == '__main__':
    main()

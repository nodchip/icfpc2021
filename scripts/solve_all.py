import sys
import argparse
import subprocess
import shutil
import json
import re
import glob
import os
import tqdm
import pandas as pd

from pathlib import Path
REPO_DIR = Path(__file__).resolve().parent.parent.resolve()
DEFAULT_SOLUTIONS_DIR = REPO_DIR / 'solutions'
DEFAULT_PROBLEMS_DIR = REPO_DIR / 'data' / 'problems'
if sys.platform == 'win32':
    EXE_DIR = REPO_DIR / 'vs' / 'x64' / 'Release'
else:
    EXE_DIR = REPO_DIR / 'src'

def main():
    parser = argparse.ArgumentParser(description='Brain Wall batch solver')
    parser.add_argument('solver_name', help='Solver to use')
    parser.add_argument('overwrite_option', default='improvement', choices=['force', 'improvement', 'never'], help='when to overwite existing files')
    parser.add_argument('--output-dir', default=DEFAULT_SOLUTIONS_DIR, type=Path)
    parser.add_argument('--problems-dir', default=DEFAULT_PROBLEMS_DIR, type=Path)
    args = parser.parse_args()

    rows = []
    for problem_json in tqdm.tqdm(glob.glob(str(args.problems_dir / '*.json'))):
        name = Path(problem_json).name
        print(name)
        mo = re.search(r'^(\d+)', name)
        if mo:
            num = int(mo.groups()[0])
            out_path = args.output_dir / f'{num}.pose.json'
        else:
            out_path = args.output_dir / (Path(problem_json).stem + '.pose.json')

        exists = out_path.is_file()

        action = 'direct'
        if args.overwrite_option == 'never':
            if exists:
                action = 'skip'
            else:
                action = 'direct'
        elif args.overwrite_option == 'force':
            action = 'direct'
        elif args.overwrite_option == 'improvement': 
            if exists:
                action = 'compare'
            else:
                action = 'direct'

        
        if action == 'skip':
            print('skip')
            rows.append({'name': name, 'result': 'skip'})
        
        elif action == 'direct':
            # direct output
            subprocess.run([str(EXE_DIR / 'solver'), 'solve', args.solver_name, str(Path(problem_json).resolve()), str(out_path)], cwd=EXE_DIR)
            print('wrote')
            rows.append({'name': name, 'result': 'wrote'})
        
        elif action == 'compare':
            tmp_out_path = out_path.with_suffix(out_path.suffix + '.tmp')
            subprocess.run([str(EXE_DIR / 'solver'), 'solve', args.solver_name, str(Path(problem_json).resolve()), str(tmp_out_path)], cwd=EXE_DIR)

            with open(out_path, 'r') as fi:
                j = json.load(fi)

            has_judge = 'meta' in j and 'judge' in j['meta']
            if not has_judge:
                tmp2_out_path = out_path.with_suffix(out_path.suffix + '.tmp')
                shutil.copyfile(out_path, tmp2_out_path)
                subprocess.run([str(EXE_DIR / 'judge'), str(Path(problem_json).resolve()), str(tmp2_out_path)], cwd=EXE_DIR)
                with open(tmp2_out_path, 'r') as fi:
                    j = json.load(fi)
                os.unlink(tmp2_out_path)
            old_is_valid = j['meta']['judge']['is_valid']
            old_dislikes = j['meta']['judge']['dislikes']
            print(old_is_valid, old_dislikes)

            with open(tmp_out_path, 'r') as fi:
                j = json.load(fi)
            new_is_valid = j['meta']['judge']['is_valid']
            new_dislikes = j['meta']['judge']['dislikes']
            print(new_is_valid, new_dislikes)
            
            improved = False
            if old_is_valid:
                if new_is_valid:
                    improved = new_dislikes < old_dislikes
                else:
                    pass
            else:
                if new_is_valid:
                    improved = True
                else:
                    improved = new_dislikes < old_dislikes # both invalid but a better score.
            
            if improved:
                shutil.move(tmp_out_path, out_path)
                print('wrote (improved)')
                rows.append({'name': name, 'result': 'wrote_improve'})
            else:
                os.unlink(tmp_out_path)
                print('skip (did not improve)')
                rows.append({'name': name, 'result': 'skip_noimprove'})

    df = pd.DataFrame(rows)
    df.to_csv('solve_all.csv', index=False)

if __name__ == '__main__':
    main()

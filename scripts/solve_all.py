import argparse
import concurrent.futures
import glob
import json
import multiprocessing as mp
import os
from sys import stderr
import pandas as pd
import re
import shutil
import subprocess
import tqdm

from common import *


def job(problem_json, args):
    name = Path(problem_json).name
    print('\n' + name)
    mo = re.search(r'^(\d+)', name)
    if mo:
        num = int(mo.groups()[0])
        out_path = args.output_dir / f'{num}.pose.json'
    else:
        out_path = args.output_dir / (Path(problem_json).stem + '.pose.json')

    solver_subcommand = 'solve'
    if args.try_globalist:
        solver_subcommand = 'try_globalist'
    print(f'subcommand: {solver_subcommand}')

    # solve.
    tmp_out_path = out_path.with_suffix(out_path.suffix + '.tmp')
    subprocess.run([str(EXE_DIR / 'solver'), solver_subcommand, args.solver_name, str(Path(problem_json).resolve()),
                    str(tmp_out_path)], cwd=EXE_DIR, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    with open(tmp_out_path, 'r') as fi:
        j = json.load(fi)
    new_is_valid = j['meta']['judge']['is_valid']
    new_dislikes = j['meta']['judge']['dislikes']

    row = {'name': name, 'new_is_valid': new_is_valid,
           'new_dislikes': new_dislikes}

    if not new_is_valid:
        print(name, 'skip (invalid)')
        os.unlink(tmp_out_path)
        return dict(row, result='invalid')

    action = 'force'
    exists = out_path.is_file()
    if args.overwrite_option == 'never':
        if exists:
            action = 'skip'
        else:
            action = 'force'
    elif args.overwrite_option == 'force':
        action = 'force'
    elif args.overwrite_option == 'improvement':
        if exists:
            action = 'compare'
        else:
            action = 'force'

    if action == 'skip':
        os.unlink(tmp_out_path)
        print(name, 'skip (never overwrite)')
        return dict(row, result='skip_never_overwrite')
    elif action == 'force':
        shutil.move(tmp_out_path, out_path)
        print(name, 'wrote')
        return dict(row, result='wrote')

    elif action == 'compare':
        with open(out_path, 'r') as fi:
            j = json.load(fi)

        has_judge = 'meta' in j and 'judge' in j['meta']
        if not has_judge:
            tmp2_out_path = out_path.with_suffix(out_path.suffix + '.tmp2')
            shutil.copyfile(out_path, tmp2_out_path)
            subprocess.run(
                [str(EXE_DIR / 'judge'), str(Path(problem_json).resolve()), str(tmp2_out_path)], cwd=EXE_DIR)
            with open(tmp2_out_path, 'r') as fi:
                j = json.load(fi)
            os.unlink(tmp2_out_path)
        old_is_valid = j['meta']['judge']['is_valid']
        old_dislikes = j['meta']['judge']['dislikes']
        row['old_is_valid'] = old_is_valid
        row['old_dislikes'] = old_dislikes

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
                # both invalid but a better score.
                improved = new_dislikes < old_dislikes

        if improved:
            shutil.move(tmp_out_path, out_path)
            print(name, 'wrote (improved)')
            return dict(row, result='wrote_improve')
        else:
            os.unlink(tmp_out_path)
            print(name, 'skip (did not improve)')
            return dict(row, result='skip_noimprove')


def main():
    parser = argparse.ArgumentParser(description='Brain Wall batch solver')
    parser.add_argument('solver_name', help='Solver to use')
    parser.add_argument('overwrite_option', default='improvement', choices=[
                        'force', 'improvement', 'never'], help='when to overwite existing files')
    parser.add_argument(
        '--try-globalist', action='store_true', help='try GLOBALIST mode')
    parser.add_argument(
        '--output-dir', default=None, type=Path)
    parser.add_argument(
        '--problems-dir', default=DEFAULT_PROBLEMS_DIR, type=Path)
    parser.add_argument('-j', '--parallel', type=int, default=mp.cpu_count())
    args = parser.parse_args()

    args.problems_dir = args.problems_dir.resolve()

    if args.output_dir is None:
        args.output_dir = SOLUTIONS_DIR / args.solver_name
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(exist_ok=True)

    rows = []

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.parallel) as executor:
        futures = [executor.submit(job, json, args) for json in glob.glob(
            str(args.problems_dir / '*.json'))]
        with tqdm.tqdm(total=len(futures)) as t:
            for future in futures:
                rows.append(future.result())
                t.update(1)

    df = pd.DataFrame(rows)
    df.to_csv('solve_all.csv', index=False)


if __name__ == '__main__':
    main()

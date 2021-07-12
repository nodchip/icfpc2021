import argparse
import optuna
import json
import subprocess
import os

from common import *

PROBLEM_ID = None


def objective(trial):
    global PROBLEM_ID
    pose_directory_path = SOLUTIONS_DIR / 'OptunaAnnealingSolver'
    pose_directory_path.mkdir(exist_ok=True)
    pose_file_path = pose_directory_path / f'{PROBLEM_ID}.pose.json'
    temporary_pose_file_path = pose_directory_path / f'{PROBLEM_ID}.pose.tmp'
    solver_subcommand = 'solve'
    solver_name = 'OptunaAnnealingSolver'
    problem_json_file_path = DEFAULT_PROBLEMS_DIR / \
        f'{PROBLEM_ID}.problem.json'
    parameters_json_file_path = EXE_DIR / f'{PROBLEM_ID}.parameters.json'

    parameters_json = {
        'initialize_pose_by_hole': trial.suggest_int('initialize_pose_by_hole', 0, 1),
        'T0': trial.suggest_float('T0', 1e0, 1e2, log=True),
        'T1': trial.suggest_float('T1', 1e-3, 1e-1, log=True),
        'single_small_change_max_delta': trial.suggest_int('single_small_change_max_delta', 1, 3),
        'shift_max_delta': trial.suggest_int('shift_max_delta', 1, 3),
        'slight_rotate_max_deg': trial.suggest_float('slight_rotate_max_deg', 1.0, 5.0),
        'vote_pow': trial.suggest_float('vote_pow', 1.0, 9.0),
        'slide_protrusion_max_delta': trial.suggest_float('slide_protrusion_max_delta', 1, 3),
        'single_small_change_probability': trial.suggest_float('single_small_change_probability', 0.0, 1.0),
        'slight_rotate_probability': trial.suggest_float('slight_rotate_probability', 0.0, 1.0),
        'shift_probability': trial.suggest_float('shift_probability', 0.0, 1.0),
        'hop_grid_probability': trial.suggest_float('hop_grid_probability', 0.0, 1.0),
        'flip_probability': trial.suggest_float('flip_probability', 0.0, 1.0),
        'slide_protrusion_probability': trial.suggest_float('slide_protrusion_probability', 0.0, 1.0),
        'protrusion_cost_coefficient0': trial.suggest_float('protrusion_cost_coefficient0', 1e-1, 1e1, log=True),
        'protrusion_cost_coefficient1': trial.suggest_float('protrusion_cost_coefficient1', 1e-3, 1e-1, log=True),
        'deformation_cost_coefficient': trial.suggest_float('deformation_cost_coefficient', 1e0, 1e2, log=True),
        'dislikes_cost_coefficient': trial.suggest_float('dislikes_cost_coefficient', 1e-6, 1e-1, log=True),
    }

    # (initialize_pose_by_hole = false かつ prohibit_unfeasible_after_feasible = true は禁止)
    if parameters_json['initialize_pose_by_hole']:
        parameters_json['prohibit_unfeasible_after_feasible'] = trial.suggest_int('prohibit_unfeasible_after_feasible', 0, 1)
    else:
        parameters_json['prohibit_unfeasible_after_feasible'] = 0

    with open(parameters_json_file_path, 'w') as file:
        json.dump(parameters_json, file)

    subprocess_args = [str(EXE_DIR / 'solver'), solver_subcommand, solver_name, str(problem_json_file_path),
                       str(temporary_pose_file_path), f'--parameters_json={parameters_json_file_path}']
    completed_process = subprocess.run(subprocess_args, cwd=EXE_DIR)
    if completed_process.returncode:
        raise Exception(completed_process)
    
    with open(temporary_pose_file_path, 'r') as file:
        temporary_pose_json = json.load(file)
    temporary_dislike = temporary_pose_json['meta']['judge']['dislikes']
    temporary_is_valid = temporary_pose_json['meta']['judge']['is_valid']

    if temporary_is_valid:
        temporary_pose_json['meta']['parameters'] = parameters_json
        if os.path.isfile(pose_file_path):
            with open(pose_file_path, 'r') as file:
                pose_json = json.load(file)
            dislike = pose_json["meta"]['judge']['dislikes']
            if dislike > temporary_dislike:
                with open(pose_file_path, 'w') as file:
                    json.dump(temporary_pose_json, file)
        else:
            with open(pose_file_path, 'w') as file:
                json.dump(temporary_pose_json, file)

    os.remove(temporary_pose_file_path)

    if temporary_is_valid:
        return temporary_dislike
    else:
        return 1e8


def main():
    global PROBLEM_ID
    parser = argparse.ArgumentParser(description='Optimize parameters.')
    parser.add_argument('--problem_id', type=int, help='Problem ID', required=True)
    args = parser.parse_args()

    PROBLEM_ID = args.problem_id

    SQLITE_DIR.mkdir(exist_ok=True)

    study = optuna.create_study(
        storage=f'sqlite:///{SQLITE_DIR}/{PROBLEM_ID}.db', study_name=f'{PROBLEM_ID}', load_if_exists=True)
    study.optimize(objective, show_progress_bar=True)


if __name__ == '__main__':
    main()

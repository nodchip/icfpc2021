#!/usr/bin/python3

import flask
import json
import math
import os
import pandas as pd
import re
import visualize

app = flask.Blueprint('problems', __name__)

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBLEMS_DIR = os.path.join(ROOT_DIR, 'data', 'problems')
SOLUTIONS_DIR = os.path.join(ROOT_DIR, 'solutions')
WWW_DIR = os.path.join(ROOT_DIR, 'www')


@app.route('/problems.html')
def show_problems():
    problems = struct_problems()
    solutions = get_all_solutions()
    problem_contexts = [make_problem_context(id, problem, solutions.get(id, []))
                        for id, problem in enumerate(problems, start=1)]

    context = {
        'problems': problem_contexts,
        'emojis': {'GLOBALIST': '🌏', 'BREAK_A_LEG': '🦵', 'WALLHACK': '🧱', 'SUPERFLEX': '🦯'},
    }
    return flask.render_template('problems.html', title='Problems', **context)


def make_problem_context(id, problem, solutions):
    def get_state(p, l):
        d, b = p['dislikes'], p['best_dislikes']
        if d is None:
            return 'secondary' if l is None else 'danger'
        if l is None or d < l:
            return 'warning'
        if d > l:
            return 'danger'
        if d == b:
            return 'success'
        return None

    img_url_path = 'images/problems/{}.problem.png'.format(id)

    # Create an image if it does not exit.
    img_file_path = os.path.join(WWW_DIR, img_url_path)
    if not os.path.isfile(img_file_path):
        img_dir_path = os.path.dirname(img_file_path)
        if not os.path.exists(img_dir_path):
            os.makedirs(img_dir_path)
        problem_file_path = os.path.join(PROBLEMS_DIR, '{}.problem.json'.format(id))
        visualize.visualize(problem_file_path, None, img_file_path)

    local_dislikes = solutions[0]['meta']['judge']['dislikes'] if solutions else None
    return {
        'id': id,
        'image': img_url_path,
        'epsilon': problem['epsilon'],
        'max_score': problem['max_score'],
        'best_dislikes': str(problem['best_dislikes']) if problem['best_dislikes'] else None,
        'dislikes': str(problem['dislikes']) if problem['dislikes'] is not None else None,
        'num_holes': len(problem['hole']),
        'num_verts': len(problem['figure']['vertices']),
        'solutions': [solution_context(problem, x) for x in solutions],
        'state': get_state(problem, local_dislikes),
        'bonuses': problem['bonuses'],
    }


def solution_context(problem, solution):
    meta = solution['meta']
    assert 'judge' in meta, '"judge" is not found'
    judge = meta['judge']
    dislikes = judge['dislikes']
    id = problem['id']
    subdir = meta['subdir']
    img_url_path = 'images/solutions/{}/{}.pose.png'.format(subdir, id)

    # Create an image if it does not exit.
    img_file_path = os.path.join(WWW_DIR, img_url_path)
    if not os.path.isfile(img_file_path):
        img_dir_path = os.path.dirname(img_file_path)
        if not os.path.exists(img_dir_path):
            os.makedirs(img_dir_path)
        problem_file_path = os.path.join(PROBLEMS_DIR, '{}.problem.json'.format(id))
        solution_file_path = os.path.join(SOLUTIONS_DIR, subdir, '{}.pose.json'.format(id))
        visualize.visualize(problem_file_path, solution_file_path,
                            img_file_path, does_draw_figure=False)

    return {
        'solver': meta['solver'],
        'subdir': subdir,
        'bonuses': solution.get('bonuses', []),
        'gained_bonuses': judge['gained_bonuses'],
        'dislikes': str(dislikes),
        'score': str(get_score(problem, dislikes)),
        'eligible': is_eligible_for_submit(solution),
        'img_path': img_url_path,
    }


def get_all_solutions():
    solutions = {}
    for base_dir, _, files in os.walk(SOLUTIONS_DIR):
        subdir = os.path.basename(base_dir)
        for filename in files:
            id = int(re.sub(r'\D', '', filename))
            solution = load_pose_json(type=subdir, id=id)
            if not solution:
                continue
            assert 'meta' in solution, 'No meta info in {}/{}.pose.json'.format(subdir, id)
            if not solution['meta']['judge']['is_valid']:
                continue
            if 'solver' not in solution['meta']:
                solution['meta']['solver'] = subdir
            solution['meta']['subdir'] = subdir
            if 'gained_bonuses' not in solution['meta']['judge']:
                solution['meta']['judge']['gained_bonuses'] = []
            if id not in solutions:
                solutions[id] = []
            solutions[id].append(solution)
    for id in solutions.keys():
        solutions[id].sort(key=lambda x: x['meta']['judge']['dislikes'] * 100 - len(x['meta']['judge']['gained_bonuses']))
    return solutions


def struct_problems():
    contest_infos = pd.read_table(os.path.join(WWW_DIR, 'contest_infos.tsv'), index_col=0)
    ids = contest_infos.index
    problems = [parse_problem(id, contest_infos.loc[id]) for id in ids]

    # Resolve bonus dependencies
    for id, problem in enumerate(problems, start=1):
        for bonus in filter(lambda x: 'problem' in x, problem.get('bonuses', [])):
            j = bonus['problem'] - 1
            if 'bonuses' not in problems[j]:
                problems[j]['bonuses'] = []
            to_bonus = {
                'bonus': bonus['bonus'],
                'from': id,
            }
            problems[j]['bonuses'].append(to_bonus)

    return problems


def parse_problem(id, web):
    dislikes = web['your dislikes']
    problem = load_problem_json(id)
    problem.update({
        'id': id,
        'best_dislikes': int(web['minimal dislikes']) if not math.isnan(web['minimal dislikes']) else None,
        'dislikes': int(dislikes) if not math.isnan(dislikes) else None,
        'max_score': get_score(problem),
    })
    return problem


def load_problem_json(id):
    try:
        with open(os.path.join(PROBLEMS_DIR, '{}.problem.json'.format(id))) as f:
            return json.load(f)
    except:
        return None


def load_pose_json(id, type='submit'):
    pose_path = os.path.join(SOLUTIONS_DIR, type, '{}.pose.json'.format(id)) 
    try:
        with open(pose_path) as f:
            return json.load(f)
    except:
        return None


def get_score(problem, dislikes=None):
    vertices = len(problem['figure']['vertices'])
    edges = len(problem['figure']['edges'])
    hole = len(problem['hole'])
    score = 1000 * math.log2(vertices * edges * hole)

    if dislikes is not None:
        best = problem['best_dislikes']
        score = score * math.sqrt((best + 1) / (dislikes + 1))

    return int(math.ceil(score))


def is_eligible_for_submit(solution):
    if not solution['meta']['judge']['is_valid']:
        return False
    return True

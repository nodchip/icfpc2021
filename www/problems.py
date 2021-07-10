#!/usr/bin/python3

import flask
import json
import math
import os
import pandas as pd
import re

app = flask.Blueprint('problems', __name__)

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBLEMS_DIR = os.path.join(ROOT_DIR, 'data', 'problems')
SOLUTIONS_DIR = os.path.join(ROOT_DIR, 'solutions')
SUBMIT_DIR = os.path.join(SOLUTIONS_DIR, 'submit')  # TODO: Drop this


@app.route('/problems.html')
def show_problems():
    # TODO: Update this TSV as frequently as possible.
    contest_infos = pd.read_table(os.path.join(ROOT_DIR, 'www', 'contest_infos.tsv'), index_col=0).loc
    ids = sorted([int(re.sub(r'\D', '', x)) for x in os.listdir(path=PROBLEMS_DIR)])
    solutions = get_all_solutions(ids)
    problems = [parse_problem(id, contest_infos[id]) for id in ids]

    # Resolve bonus dependencies
    for i, problem in enumerate(problems):
        from_id = i + 1
        for bonus in problem.get('bonuses', []):
            if 'problem' not in bonus:
                continue
            j = bonus['problem'] - 1
            if 'bonuses' not in problems[j]:
                problems[j]['bonuses'] = []
            to_bonus = {
                'bonus': bonus['bonus'],
                'from': from_id,
            }
            problems[j]['bonuses'].append(to_bonus)

    problem_contexts = []
    for i, problem in enumerate(problems):
        id = i + 1
        def get_state(p, l):
            d = p['dislikes']
            b = p['best_dislikes']
            if d is None:
                if l is not None:
                    return 'danger'
            else:
                if l is None or d < l:
                    return 'warning'
                if d > l:
                    return 'danger'
                if d == b:
                    return 'success'
            return None

        local_dislikes = solutions[id][0]['meta']['judge']['dislikes'] if solutions[id] else None
        context = {
            'id': id,
            'image': 'images/{}.problem.png'.format(id),
            'epsilon': problem['epsilon'],
            'max_score': problem['max_score'],
            'best_dislikes': str(problem['best_dislikes']),
            'dislikes': str(problem['dislikes']) if problem['dislikes'] is not None else None,
            'num_holes': len(problem['hole']),
            'num_verts': len(problem['figure']['vertices']),
            'solutions': [solution_context(problem, x) for x in solutions[id]],
            'state': get_state(problem, local_dislikes),
            'bonuses': problem['bonuses'],
        }
        problem_contexts.append(context)

    context = {
        'problems': problem_contexts,
        'emojis': {'GLOBALIST': '🌏', 'BREAK_A_LEG': '🦵'},
    }
    return flask.render_template('problems.html', title='Problems', **context)


def solution_context(problem, solution):
    meta = solution['meta']
    judge = meta.get('judge', None)

    context = {
        'solver': meta['solver'],
        'subdir': meta['subdir'],
    }

    if judge:
        mine = meta['judge']['dislikes']
        context['dislikes'] = str(mine)
        context['score'] = str(get_score(problem, mine))
        context['eligible'] = is_eligible_for_submit(solution)
 
    return context


def get_all_solutions(ids):
    solutions = dict(zip(ids, [[] for _ in ids]))
    for subdir, _, files in os.walk(SOLUTIONS_DIR):
        if not files:
            continue
        type = os.path.basename(subdir)
        for filename in files:
            id = int(re.sub(r'\D', '', filename))
            solution = load_pose_json(type=type, id=id)
            if not solution:
                continue
            assert 'meta' in solution, 'No meta info in {}/{}.pose.json'.format(type, id)
            if not solution['meta']['judge']['is_valid']:
                continue
            if 'solver' not in solution['meta']:
                solution['meta']['solver'] = type
            solution['meta']['subdir'] = type
            solutions[id].append(solution)
    for id in ids:
        solutions[id] = sorted(solutions[id], key=lambda x: x['meta']['judge']['dislikes'])
    return solutions


def parse_problem(id, web):
    dislikes = web['your dislikes']
    problem = load_problem_json(id)
    problem.update({
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
    # if bonus exists, problem should be >= 0
    if 'bonuses' in solution:
        for bonus in solution['bonuses']:
            if bonus.get('problem', -1) < 0:
                return False
    return True

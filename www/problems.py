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

# TODO: Update this CSV as frequently as possible.
contest_infos = pd.read_csv(os.path.join(ROOT_DIR, 'www', 'minimal_dislikes.csv'), index_col=0).loc


@app.route('/problems.html')
def show_problems():
    ids = sorted([int(re.sub(r'\D', '', x)) for x in os.listdir(path=PROBLEMS_DIR)])
    solutions = get_all_solutions(ids)
    problem_contexts = []

    for id in ids:
        problem = load_problem_json(id)
        problem['dislikes'] = contest_infos[id]['minimal dislikes']
        problem['max_score'] = get_score(problem)

        context = {
            'id': id,
            'image': 'images/{}.problem.png'.format(id),
            'epsilon': problem['epsilon'],
            'score': problem['max_score'],
            'dislikes': problem['dislikes'],
            'solutions': [solution_context(problem, x) for x in solutions[id]],
        }
        problem_contexts.append(context)

    context = {
        'problems': problem_contexts,
    }
    return flask.render_template('problems.html', title='Problems', **context)


def solution_context(problem, solution):
    meta = solution['meta']
    judge = meta.get('judge', None)

    context = {
        'solver': meta['solver'],
    }

    if judge:
        mine = meta['judge']['dislikes']
        context['dislikes'] = str(mine)
        context['score'] = str(get_score(problem, mine, problem['max_score']))
 
    return context



def get_all_solutions(ids):
    solutions = dict(zip(ids, [[] for _ in ids]))
    for subdir, _, files in os.walk(SOLUTIONS_DIR):
        if not files:
            continue
        type = os.path.basename(subdir)
        for filename in files:
            id = int(re.sub(r'\D', '', filename))
            pose = load_pose_json(type=type, id=id)
            if not pose:
                continue
            if 'meta' not in pose:
                pose['meta'] = {}
            if 'solver' not in pose['meta']:
                pose['meta']['solver'] = type
            solutions[id].append(pose)
    for id in ids:
        solutions[id] = sorted(solutions[id], key=lambda x: x['meta']['judge']['dislikes'])
    return solutions


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


def get_score(problem, dislikes=None, best=None):
    vertices = len(problem['figure']['vertices'])
    edges = len(problem['figure']['edges'])
    hole = len(problem['hole'])
    score = 1000 * math.log2(vertices * edges * hole)

    if dislikes is not None and best is not None:
        score = score * math.sqrt((best + 1) / (dislikes + 1))

    return int(math.ceil(score))

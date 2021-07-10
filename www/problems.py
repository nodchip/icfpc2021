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
SUBMIT_DIR = os.path.join(ROOT_DIR, 'solutions', 'submit')

# TODO: Update this CSV as frequently as possible.
contest_infos = pd.read_csv(os.path.join(ROOT_DIR, 'www', 'minimal_dislikes.csv'), index_col=0).loc


@app.route('/problems.html')
def show_problems():
    ids = sorted([int(re.sub(r'\D', '', x)) for x in os.listdir(path=PROBLEMS_DIR)])
    problems = []
    for id in ids:
        problem = load_problem_json(id)

        context = {
            'id': id,
            'image': 'images/{}.problem.png'.format(id),
            'epsilon': problem['epsilon'],
            'score': {
                'upperbound': get_score(problem),
            },
            'dislikes': {
                'best': contest_infos[id]['minimal dislikes'],
            },
        }

        # Problem dynamic states
        pose = load_pose_json(id)
        if pose and pose['meta']['judge']['is_valid']:
            mine = pose['meta']['judge']['dislikes']
            context['dislikes']['submit'] = str(mine)
            context['score']['submit'] = str(get_score(problem, mine, contest_infos[id]['minimal dislikes']))

        problems.append(context)

    context = {
        'problems': problems,
    }
    return flask.render_template('problems.html', title='Problems', **context)


def load_problem_json(id):
    try:
        with open(os.path.join(PROBLEMS_DIR, '{}.problem.json'.format(id))) as f:
            return json.load(f)
    except:
        return None


def load_pose_json(id):
    pose_path = os.path.join(SUBMIT_DIR, '{}.pose.json'.format(id)) 
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

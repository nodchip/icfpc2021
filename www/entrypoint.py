#!/usr/bin/python3

import flask
import os
import json
import re
import math
import pandas as pd

app = flask.Flask(__name__)

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBLEMS_DIR = os.path.join(ROOT_DIR, 'data', 'problems')
SUBMIT_DIR = os.path.join(ROOT_DIR, 'solutions', 'submit')

df_minimial_dislikes = pd.read_csv(os.path.join(ROOT_DIR, 'www', 'minimal_dislikes.csv'), index_col=0)

@app.route('/')
def home():
    return flask.render_template('home.html', title='ICFPC2021 sanma')

@app.route('/problems.html')
def show_problems():
    ids = sorted([int(re.sub(r'\D', '', x)) for x in os.listdir(path=PROBLEMS_DIR)])
    problems = []
    for id in ids:
        context = {
            'id': id,
            'image': 'images/{}.problem.png'.format(id),
        }
        with open(os.path.join(PROBLEMS_DIR, '{}.problem.json'.format(id))) as f:
            prob = json.load(f)
            vertices = len(prob['figure']['vertices'])
            edges = len(prob['figure']['edges'])
            hole = len(prob['hole'])
            context['score_upperbound'] = int(math.ceil(1000 * math.log2(vertices * edges * hole)))
            context['epsilon'] = prob['epsilon']

        best = df_minimial_dislikes.loc[id]['minimal dislikes']
        context['dislikes_best'] = best

        context['score_submit'] = 'no solution'
        context['dislikes_submit'] = 'no solution'
        pose_path = os.path.join(SUBMIT_DIR, '{}.pose.json'.format(id)) 
        if os.path.isfile(pose_path):
            with open(pose_path) as f:
                prob = json.load(f)
                try:
                    if prob['meta']['judge']['is_valid']:
                        mine = prob['meta']['judge']['dislikes']
                        context['dislikes_submit'] = mine
                        context['score_submit'] = int(math.ceil(1000 * math.log2(vertices * edges * hole) * math.sqrt((best + 1) / (mine + 1))))
                    else:
                        context['dislikes_submit'] = 'infeasible'
                except:
                    context['dislikes_submit'] = 'missing meta'

        problems.append(context)

    context = {
        'problems': problems,
    }
    return flask.render_template('problems.html', title='Problems', **context)

@app.route('/images/<path:path>')
def send_images(path):
    return flask.send_from_directory('./images', path)


if __name__ == "__main__":
    app.run(debug=True)

#!/usr/bin/python3

import flask
import os
import re

app = flask.Flask(__name__)

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROBLEMS_DIR = os.path.join(ROOT_DIR, 'data', 'problems')


@app.route('/')
def home():
    return flask.render_template('home.html', title='ICFPC2021 sanma')


@app.route('/problems.html')
def show_problems():
    ids = sorted([int(re.sub(r'\D', '', x)) for x in os.listdir(path=PROBLEMS_DIR)])

    problems = [{
        'id': id,
        'image': 'images/{}.problem.png'.format(id),
        'file': os.path.join(PROBLEMS_DIR, '{}.problem.json'.format(id)),
    } for id in ids]
    context = {
        'problems': problems,
    }
    return flask.render_template('problems.html', title='Problems', **context)

@app.route('/images/<path:path>')
def send_images(path):
    return flask.send_from_directory('./images', path)


if __name__ == "__main__":
    app.run(debug=True)

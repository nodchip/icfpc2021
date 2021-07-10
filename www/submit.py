#!/usr/bin/python3

import flask
import json
import math
import os
import pandas as pd
import re
import requests

app = flask.Blueprint('submit', __name__)

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOLUTIONS_DIR = os.path.join(ROOT_DIR, 'solutions')
API_TOKEN = 'f086e0bb-cfeb-495d-9f1d-240ea717b30b'

@app.route('/submit', methods=['POST'])
def submit_pose():
    data = flask.request.get_data().decode()
    data = json.loads(data)
    id = data['id']
    solver = data['solver']
    if solver == 'ManualSolver':
        solver = 'by_hands'
    solution = json.dumps(load_pose_json(type=solver, id=id))

    headers = {'Authorization': 'Bearer {}'.format(API_TOKEN)}
    url = 'https://poses.live/api/problems/{}/solutions'.format(id)
    response = requests.post(url, headers=headers, data=solution)
    print(response)
    return response.text


def load_pose_json(id, type='submit'):
    pose_path = os.path.join(SOLUTIONS_DIR, type, '{}.pose.json'.format(id)) 
    try:
        with open(pose_path) as f:
            return json.load(f)
    except:
        return None

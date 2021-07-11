#!/usr/bin/python3

import flask
import json
import os
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
    subdir = data['subdir']
    if subdir == 'ManualSolver':
        subdir = 'by_hands'
    solution = load_pose_json(subdir=subdir, id=id)
    assert solution is not None, data
    solution = json.dumps(solution)

    headers = {'Authorization': 'Bearer {}'.format(API_TOKEN)}
    url = 'https://poses.live/api/problems/{}/solutions'.format(id)
    response = requests.post(url, headers=headers, data=solution)
    if response.status_code != 200:
        print('>', solution)
        print(response.text)
    return response.text


def load_pose_json(id, subdir='submit'):
    pose_path = os.path.join(SOLUTIONS_DIR, subdir, '{}.pose.json'.format(id)) 
    try:
        with open(pose_path) as f:
            return json.load(f)
    except:
        print('Fail to open {} {}'.format(subdir, pose_path))
        return None

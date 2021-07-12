import argparse
import concurrent.futures
import glob
import json
import multiprocessing as mp
import os
from sys import stderr
import re
import shutil
import subprocess
import tqdm
import csv
import urllib.request
import requests

from common import *


def main():
    parser = argparse.ArgumentParser(description='Brain Wall batch solver')
    parser.add_argument('token', help='Jenkins API Token.')
    parser.add_argument('--stop', action='store_true',
                        help='Stop the current jobs.')
    parser.add_argument('--start', action='store_true', help='Start jobs.')
    parser.add_argument('--solver_name', type=int, help='Solver name')
    args = parser.parse_args()

    if args.stop:
        # jenkins — Jenkinsから、現在実行中のジョブのリストをJSONで取得するにはどうすればよいですか？ https://www.it-swarm-ja.com/ja/jenkins/jenkins%E3%81%8B%E3%82%89%E3%80%81%E7%8F%BE%E5%9C%A8%E5%AE%9F%E8%A1%8C%E4%B8%AD%E3%81%AE%E3%82%B8%E3%83%A7%E3%83%96%E3%81%AE%E3%83%AA%E3%82%B9%E3%83%88%E3%82%92json%E3%81%A7%E5%8F%96%E5%BE%97%E3%81%99%E3%82%8B%E3%81%AB%E3%81%AF%E3%81%A9%E3%81%86%E3%81%99%E3%82%8C%E3%81%B0%E3%82%88%E3%81%84%E3%81%A7%E3%81%99%E3%81%8B%EF%BC%9F/1071072010/
        url = f'http://hnoda-dt2:8080/job/ICFPC2021.optimize_parameters.2021-07-11/api/json'
        with urllib.request.urlopen(url) as f:
            jenkins = json.loads(f.read())
        for build in jenkins['builds']:
            url = f'{build["url"]}stop'
            print(url)
            response = requests.post(url, auth=('hnoda', args.token))
            print(response.status_code)

    if args.start:
        # CSV ファイルや TSV ファイルを読み込む (csv.reader) | まくまくPythonノート https://maku77.github.io/python/io/csv.html
        with open(REPO_DIR / 'www' / 'contest_infos.tsv', encoding='utf-8', newline='') as f:
            for row in csv.reader(f, delimiter='\t'):
                problem_id = row[0]
                dislikes = row[1]
                if dislikes == 0:
                    continue
                # Parameterized Build - Jenkins - Jenkins Wiki https://wiki.jenkins.io/display/JENKINS/Parameterized+Build
                url = f'http://hnoda-dt2:8080/job/ICFPC2021.optimize_parameters.2021-07-11/buildWithParameters?problem_id={problem_id}&solver_name={args.solver_name}'
                print(url)
                requests.post(url, auth=('hnoda', args.token))


if __name__ == '__main__':
    main()

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
    parser.add_argument('--first_build_number', type=int,
                        help='First build number to be stopped.')
    parser.add_argument('--last_build_number', type=int,
                        help='Last build number to be stopped.')
    parser.add_argument('--start', action='store_true', help='Start jobs.')
    parser.add_argument('--solver_name', type=str, help='Solver name')
    args = parser.parse_args()

    if args.stop:
        for build_number in range(args.first_build_number, args.last_build_number + 1):
            url = f'http://jenkins:8080/job/ICFPC2021.optimize_parameters.2021-07-11/{build_number}/stop'
            print(url)
            response = requests.post(url, auth=('hnoda', args.token))
            print(response.status_code)

    if args.start:
        # CSV ファイルや TSV ファイルを読み込む (csv.reader) | まくまくPythonノート https://maku77.github.io/python/io/csv.html
        with open(REPO_DIR / 'www' / 'contest_infos.tsv', encoding='utf-8', newline='') as f:
            for row in csv.reader(f, delimiter='\t'):
                problem_id = row[0]
                if problem_id == 'problem':
                    continue
                dislikes = row[1]
                # 満点が求まっている問題についてはソルバーを実行しない。
                # ただし、dislikesが空欄の問題は、有効な出力が得られていないので、
                # ソルバーを実行する。
                if dislikes == '0':
                    continue
                # Parameterized Build - Jenkins - Jenkins Wiki https://wiki.jenkins.io/display/JENKINS/Parameterized+Build
                url = f'http://jenkins:8080/job/ICFPC2021.optimize_parameters.2021-07-11/buildWithParameters?problem_id={problem_id}&solver_name={args.solver_name}'
                print(url)
                requests.post(url, auth=('hnoda', args.token))


if __name__ == '__main__':
    main()

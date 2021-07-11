#!/usr/bin/python3
import os
import sys
import glob
import requests
from collections import defaultdict
import datetime
import time
import pandas as pd
import matplotlib.pyplot as plt
import lxml.html

def get_scoreboard():
    url = 'https://poses.live/teams'
    response = requests.get(url)
    html = lxml.html.fromstring(response.content)

    now = datetime.datetime.now()
    print('capture ', now)

    rows = []
    for tr in html.xpath('//tr'):
        row = []
        for td in tr.xpath('td'):
            row.append(td.text)
        if len(row) == 3:
            place, team, score = row
            place = int(place)
            score = int(score)
            rows.append({'place': place, 'team': team, 'score': score, 'captured_at': now})

    df = pd.DataFrame(rows)
    os.makedirs('scoreboards', exist_ok=True)
    df.to_csv(f'scoreboards/{now.strftime("%Y%m%d_%H%M%S.csv")}.csv')

def summarize():

    team_to_scoreseries = defaultdict(list)
    team_to_placeseries = defaultdict(list)
    for f in sorted(glob.glob('scoreboards/*.csv')):
        df = pd.read_csv(f)
        for team, place, score in zip(df.team.values, df.place.values, df.score.values):
            team_to_scoreseries[team].append(score)
            team_to_placeseries[team].append(place)

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    for team, scores in team_to_scoreseries.items():
        if len(team_to_placeseries[team]) == 1:
            best_place = team_to_placeseries[team][0]
        else:
            best_place = min(*team_to_placeseries[team])
        curr_place = team_to_placeseries[team][-1]
        if best_place < 15 or team == 'sanma':
            label = f'{team} best {best_place} curr {curr_place}'
            cax, = ax.plot(scores, '.-')
            ax.text(len(scores), scores[-1], label, horizontalalignment='left', fontsize=8, color=cax.get_color())
    ax.set_xlim(0, ax.get_xlim()[1] + 2)
    ax.set_xlabel('Time')
    ax.set_ylabel('Score')
    ax.set_title(f'Scoreboard {df.iloc[-1].captured_at}')
    plt.show()
    #fig.savefig('scoreboard.png')

def main():
    while True:
        get_scoreboard()
        print('sleep..')
        time.sleep(5 * 60)

if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'get':
        main()
    else:
        summarize()

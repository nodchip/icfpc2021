#!/usr/bin/python3

import json
import os
import glob
from graphviz import Digraph

def main():
    dg = Digraph(format='png')
    for file in glob.glob('../data/problems/*.json'):
        filename = os.path.basename(file)
        problem = filename[0:filename.find('.')]
        with open(file) as f:
            data = json.load(f)
            for bonus in data['bonuses']:
                tp = f"{bonus['problem']}"
                tb = f"{bonus['bonus']}"
                dg.node(tp, style="filled", fillcolor=("red" if tb == "GLOBALIST" else "blue"))
                dg.edge(problem, tp)
    dg.render('./dgraph', view=True)

main()

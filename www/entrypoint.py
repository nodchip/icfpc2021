#!/usr/bin/python3

import flask
import problems
import submit

app = flask.Flask(__name__)
app.register_blueprint(problems.app)
app.register_blueprint(submit.app)

@app.route('/')
def home():
    return flask.render_template('home.html', title='ICFPC2021 sanma')


@app.route('/images/<path:path>')
def send_images(path):
    return flask.send_from_directory('./images', path)


if __name__ == "__main__":
    app.run(debug=True)

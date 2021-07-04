from flask import Flask, render_template
app = Flask(__name__)


@app.route('/')
def home():
    return render_template('home.html', title='ICFPC2020 sanma team home page')


if __name__ == "__main__":
    app.run(debug=True)

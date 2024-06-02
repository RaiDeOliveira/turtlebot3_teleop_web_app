from flask import Flask, render_template

# Flask definitions
app = Flask(__name__)

@app.route('/')
def hello_world():
    return render_template('index.html')

def main():
    app.run('0.0.0.0', 8000, True)
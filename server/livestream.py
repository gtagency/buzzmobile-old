from bottle import route, run, template, static_file


@route('/')
def index():
    return template("index.html")


@route('/img/<filename:path>')
def image(filename):
    return static_file(filename, root="./static")


run(host="0.0.0.0", port=80)

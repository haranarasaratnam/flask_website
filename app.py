'''

- to run $python app.py

Updates:

- branched out form dev1 (working copy with ebooks) to dev2 on 1/7/24

- venv: web

#todo

- https://allenfrostline.com




------------------------------
12/8/24
- Refactored app.py to scale the routes
- Added control and coding, removed contact form


10/27/24

- added new JS script to base.html for a proper math equation rendering (MathJax) -->
check out EN with #mathjax for cheatsheet

10/26/24
- ported to PythonAnywhere


1/4/24
- added estimation with 2 chaps

5/17/24
- added js to render rotating quotes for portfolio page

4/29/24
- added flask_mail to contact form (flask_mail not able to be installed on bluehost server due to missing  _ctypes
by-passed this with SMTPLIB


4/28/24
- added teaching page

4/26/24
- added code to export files from local to remote using scp (utils/export)
(#todo- move config info to zshrc)

4/25/24
- removed adsense

4/23/24
- applied for Adsense

4/21/24

- Completed refactoring using Jinga2 block

4/19/24
- refactored using block and extends

- added footer

'''
from datetime import datetime
from pathlib import Path

from flask import Flask, render_template, url_for

app = Flask(__name__)


def _last_updated(template):
    """Return the template file's mtime as YYYY-MM-DD, or None if unreadable."""
    path = Path(app.root_path) / app.template_folder / template.lstrip('./')
    try:
        return datetime.fromtimestamp(path.stat().st_mtime).strftime('%Y-%m-%d')
    except OSError:
        return None


# Define routes and their corresponding templates
routes_main = {
    '/index': {'template': './index.html', 'endpoint': 'index'},
    '/': {'template': './index.html', 'endpoint': 'home'},
    '/portfolio': {'template': './portfolio.html', 'endpoint': 'portfolio'},
    '/estimation': {'template': './estimation.html', 'endpoint': 'estimation'},
    '/control': {'template': './control.html', 'endpoint': 'control'},
    '/coding': {'template': './coding.html', 'endpoint': 'coding'},
    # '/resume': {'template': './resume.html', 'endpoint': 'resume'},  # intentionally hidden — see CLAUDE.md
}

# Define routes and their corresponding templates
routes_estimation = {
    '/estimation/intro': {'template': './estimation/intro.html', 'endpoint': 'intro'},
    '/estimation/estimate_dc_motor': {'template': './estimation/estimate_dc_motor.html', 'endpoint': 'dc_motor'},
    '/estimation/ckf': {'template': './estimation/ckf.html', 'endpoint': 'ckf'},
    '/estimation/sckf': {'template': './estimation/sckf.html', 'endpoint': 'sckf'},
    '/estimation/application_1': {'template': './estimation/application_1.html', 'endpoint': 'application_1'},
    '/estimation/application_2': {'template': './estimation/application_2.html', 'endpoint': 'application_2'},
    '/estimation/reference': {'template': './estimation/reference.html', 'endpoint': 'reference'},
}

# Define routes and their corresponding templates
routes_control = {
    '/control/lqr_for_dc_motor': {'template': './control/lqr_for_dc_motor.html', 'endpoint': 'lqr_for_dc_motor'},
    '/control/lqr_for_pendulum': {'template': './control/lqr_for_pendulum.html', 'endpoint': 'lqr_for_pendulum'},
    '/control/matlab_commands': {'template': './control/matlab_commands.html', 'endpoint': 'matlab_commands'},
}

# Define routes and their corresponding templates
routes_coding = {
    '/oop': {'template': './coding/oop.html', 'endpoint': 'oop'},
}


def route_2_render_template(routes):
    # Dynamically create routes with proper endpoint names.
    # The default-argument trick (template=data['template']) captures
    # the value per-iteration; closures alone would late-bind to whatever
    # data['template'] held on the *last* loop iteration.
    for route, data in routes.items():
        app.add_url_rule(
            route,
            endpoint=data['endpoint'],
            view_func=lambda template=data['template']: render_template(
                template, last_updated=_last_updated(template)
            ),
        )

route_2_render_template(routes_main)
route_2_render_template(routes_estimation)
route_2_render_template(routes_control)
route_2_render_template(routes_coding)


# @app.route('/')
# @app.route('/index')
# def index():
#     return render_template('./index.html', login=True)


# @app.route('/portfolio')
# def portfolio():
#     return render_template('./portfolio.html')
#
#
#
# @app.route('/resume')
# def resume():
#     return render_template('./resume.html')
#
# @app.route('/estimation/intro')
# def intro():
#     return render_template('./estimation/intro.html')
#
#
# @app.route('/estimation/ckf')
# def ckf():
#     return render_template('./estimation/ckf.html')
#
#
# @app.route('/estimation/sckf')
# def sckf():
#     return render_template('./estimation/sckf.html')
#
#
# @app.route('/estimation/application_1')
# def application_1():
#     return render_template('./estimation/application_1.html')
#
#
# @app.route('/estimation/application_2')
# def application_2():
#     return render_template('./estimation/application_2.html')
#
#
# @app.route('/estimation/reference')
# def reference():
#     return render_template('./estimation/reference.html')


# @app.route('/control')
# def control():
#     return render_template('./control.html')


# @app.route('/control/lqe')
# def lqe():
#     return render_template('./control/lqr_for_dc_motor.html')


# @app.route('/coding')
# def coding():
#     return render_template('./coding.html')
#
#
# @app.route('/estimation')
# def estimation():
#     return render_template('./estimation.html')


#
# @app.route('/contact', methods=['GET', 'POST'])
# def contact():
#     if request.method == 'POST':
#         name = request.form['name']
#         email = request.form['email']
#         subject = request.form['subject']
#         message = request.form['message']
#
#         #msg = Message(f"'A new user feedback: '{subject}", recipients=[app.config['MAIL_DEFAULT_SENDER']])
#         #msg.body = f'Name: {name}\nEmail: {email}\n\n{message}'
#         #mail.send(msg)
#
#         # Set up the email message
#         msg = MIMEText(message)
#         msg['Subject'] = f'{name}: {subject}'
#         msg['From'] = email
#         msg['To'] = app.config['MAIL_DEFAULT_SENDER']
#
#         # Connect to the SMTP server and send the message
#         server = smtplib.SMTP(app.config['MAIL_SERVER'], app.config['MAIL_PORT'])
#         server.starttls()
#         server.login(app.config['MAIL_DEFAULT_SENDER'], app.config['MAIL_PASSWORD'])
#         server.sendmail(email, app.config['MAIL_DEFAULT_SENDER'], msg.as_string())
#         server.quit()
#
#         return redirect(url_for('thankyou'))
#
#     return render_template('./contact.html')
#
# @app.route('/thankyou')
# def thankyou():
#     message = 'Thank you for contacting me!'
#     return f'<h1>{message}</h1><meta http-equiv="refresh" content="2; url=/">'  # after 2 sec, go home




    
if __name__ == '__main__':
    app.run(host='127.0.0.1', port=8080, debug=True)
    
'''

- to run $python app.py

Updates:

- branched out form dev1 (working copy with ebooks) to dev2 on 1/7/24

- venv: web

#todo

- https://allenfrostline.com




------------------------------

10/27/24

- added new JS script to base.html for a proper math equation rendering (MathJax) -->
check out EN with #mathjax for cheatsheet

10/26/24
- ported to PythonAnywhere


1/4/24
- added estimation with 2 chaps

5/17
- added js to render rotating quotes for portfolio page

4/29
- added flask_mail to contact form (flask_mail not able to be installed on bluehost server due to missing  _ctypes
by-passed this with SMTPLIB


4/28
- added teaching page

4/26
- added code to export files from local to remote using scp (utils/export)
(#todo- move config info to zshrc)

4/25
- removed adsense

4/23
- applied for Adsense

4/21

- Completed refactoring using Jinga2 block

4/19
- refactored using block and extends

- added footer

'''
from flask import Flask, render_template, url_for, request, redirect
#from flask_mail import Mail, Message
import smtplib
from email.mime.text import MIMEText

app = Flask(__name__)
#app.config.from_pyfile('config.py')

#mail = Mail(app)


@app.route('/')
@app.route('/index')
def index():
    return render_template('./index.html', login=True)


@app.route('/portfolio')
def portfolio():
    return render_template('./portfolio.html')



@app.route('/resume')
def resume():
    return render_template('./resume.html')


@app.route('/contact', methods=['GET', 'POST'])
def contact():
    if request.method == 'POST':
        name = request.form['name']
        email = request.form['email']
        subject = request.form['subject']
        message = request.form['message']

        #msg = Message(f"'A new user feedback: '{subject}", recipients=[app.config['MAIL_DEFAULT_SENDER']])
        #msg.body = f'Name: {name}\nEmail: {email}\n\n{message}'
        #mail.send(msg)

        # Set up the email message
        msg = MIMEText(message)
        msg['Subject'] = f'{name}: {subject}'
        msg['From'] = email
        msg['To'] = app.config['MAIL_DEFAULT_SENDER']

        # Connect to the SMTP server and send the message
        server = smtplib.SMTP(app.config['MAIL_SERVER'], app.config['MAIL_PORT'])
        server.starttls()
        server.login(app.config['MAIL_DEFAULT_SENDER'], app.config['MAIL_PASSWORD'])
        server.sendmail(email, app.config['MAIL_DEFAULT_SENDER'], msg.as_string())
        server.quit()

        return redirect(url_for('thankyou'))

    return render_template('./contact.html')

@app.route('/thankyou')
def thankyou():
    message = 'Thank you for contacting me!'
    return f'<h1>{message}</h1><meta http-equiv="refresh" content="2; url=/">'  # after 2 sec, go home


@app.route('/estimation')
def estimation():
    return render_template('./estimation.html')
    
@app.route('/estimation/kf')
def kf():
    return render_template('./estimation/kf.html')

@app.route('/estimation/ckf')
def ckf():
    return render_template('./estimation/ckf.html')

@app.route('/estimation/sckf')
def sckf():
    return render_template('./estimation/sckf.html')

@app.route('/estimation/application_1')
def application_1():
    return render_template('./estimation/application_1.html')

@app.route('/estimation/application_2')
def application_2():
    return render_template('./estimation/application_2.html')


@app.route('/estimation/reference')
def reference():
    return render_template('./estimation/reference.html')

@app.route('/control')
def control():
    return render_template('./control.html')

@app.route('/coding')
def coding():
    return render_template('./coding.html')

    
if __name__ == '__main__':
    app.run(debug=True)
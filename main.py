#!/usr/bin/env python2

import slacker
from slackbot.bot import Bot, respond_to
from slackbot_settings import API_TOKEN
import cv2
import tempfile

DEFAULT_CHANNEL = u'raspberrypi_channel'
slacker = slacker.Slacker(API_TOKEN)

def get_username(message):
    if 'user' in message.body:
        res1 = slacker.users.info(message.body['user']).body
        if res1['ok']:
            return res1['user']['name']
    elif 'username' in message.body:
        return message.body['username']

def post_slack_message(text, channel=DEFAULT_CHANNEL):
    slacker.chat.post_message(channel=channel, text=text, as_user=True)

@respond_to('hello')
def hello(message):
    username = get_username(message)
    channel = message.body['channel']
    post_slack_message('Hello, {}!'.format(username), channel=channel)

@respond_to('photo')
def photo(message):
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    
    if frame is None:
        post_slack_message('Could not take a picture ;(', channel=message.body['channel'])
    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=True) as tf:
        print ('Writing jpeg file '+tf.name+'.')
        try:
            cv2.imwrite(tf.name, frame)
        except Exception as e:
            print (e)
            post_slack_message('Could not convert the picture ;(', channel=message.body['channel'])
            return
        slacker.files.upload(tf.name, channels=message.body['channel'])

bot = Bot()
bot.run()

#!/usr/bin/env python2

import slacker
from slackbot.bot import Bot, respond_to
from slackbot_settings import API_TOKEN
import cv2
import tempfile

DEFAULT_CHANNEL = u'raspberrypi_channel'
slacker = slacker.Slacker(API_TOKEN)
video_capture = cv2.VideoCapture(0)

def get_username(message):
    if 'user' in message.body:
        res1 = slacker.users.info(message.body['user']).body
        if res1['ok']:
            return res1['user']['name']
    elif 'username' in message.body:
        return message.body['username']

def post_text(text, channel=DEFAULT_CHANNEL):
    slacker.chat.post_message(channel=channel, text=text, as_user=True)

def post_image(image, channel=DEFAULT_CHANNEL):
    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=True) as f:
        cv2.imwrite(f.name, image)
        slacker.files.upload(f.name, channels=channel)

@respond_to('hello')
def hello(message):
    username = get_username(message)
    channel = message.body['channel']
    post_text('Hello, {}!'.format(username), channel=channel)

@respond_to('photo')
def photo(message):
    try:
        # discard old frames from the buffer
        for _ in range(5):
            video_capture.grab()
        ret, frame = video_capture.read()
        post_image(frame, channel=message.body['channel'])
    except Exception as e:
        print e
        post_text('Could not take a picture ;(', channel=message.body['channel'])

bot = Bot()
bot.run()

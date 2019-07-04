#!/usr/bin/env python2
# -*- encoding: utf-8 -*-

import time
import slacker
import os
from slackbot.bot import Bot, respond_to, default_reply
import cv2
import tempfile
import signal
import slackbot_settings

channel_id = u'raspberrypi_channel'
threads = {}

oldest = unicode(time.time()-120)
slacker = slacker.Slacker(slackbot_settings.API_TOKEN)

def get_username(message):
    if 'user' in message:
        res1 = slacker.users.info(message['user']).body
        if res1['ok']:
            return res1['user']['name']
    elif 'username' in message:
        return message['username']

def post_slack_message(text, channel=channel_id):
    slacker.chat.post_message(channel=channel, text=text, as_user=True)

#@respond_to('pan_camera (.*)')
#def pan_camera(message, arg):
#    pass

#@respond_to('tilt_camera (.*)')
#def pan_camera(message, arg):
#    pass

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

def sigint_handler(signum, frame):
    #rospy.signal_shutdown('Manual shutdown')
    time.sleep(1)
    quit()
signal.signal(signal.SIGINT, sigint_handler)

#rospy.init_node('slack_message')
#rospy.on_shutdown(lambda: os._exit(0))

bot = Bot()
bot.run()

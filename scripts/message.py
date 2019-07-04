#!/usr/bin/env python2
# -*- encoding: utf-8 -*-

import rospy
import time
import slacker
import os
from slackbot.bot import Bot, respond_to, default_reply
from hsrb_slackbot.msg import Message
from sensor_msgs.msg import Image
from tmc_msgs.msg import Voice, TalkRequestActionGoal
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf2_ros
import tempfile
import signal
import slackbot_settings
from common import HSRB_Xtion
from common.speech import DefaultTTS
from pocketsphinx_jsgf import PocketSphinxClient
#from deepdream.srv import *
import hsrb_interface
robot = hsrb_interface.Robot()

channel_id = u'hsr_channel'
photo_topic = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
talk_topic = '/talk_request'
talk_goal = '/talk_request_action/goal'
listen_topic = '/sphinx_speech'
threads = {}

oldest = unicode(time.time()-120)
slacker = slacker.Slacker(slackbot_settings.API_TOKEN)
bridge = CvBridge()

def to_ros_msg(message):
    msg = Message()
    msg.header.stamp = rospy.Time.from_sec(float(message['ts']))
    if 'user' in message:
        res1 = slacker.users.info(message['user']).body
        if res1['ok']:
            msg.user = res1['user']['name']
    elif 'username' in message:
        msg.user = message['username']
    if 'subtype' in message:
        msg.subtype = message['subtype']
    if 'type' in message:
        msg.type = message['type']
    if 'text' in message:
        msg.text = message['text']
    return msg

def post_slack_message(text, channel=channel_id):
    slacker.chat.post_message(channel=channel, text=text, as_user=True)

@respond_to('say (.*)')
def say(message, arg):
    tts.say(arg)

@respond_to('say_ja (.*)')
def say_ja(message, arg):
    pub = rospy.Publisher('talk_request', Voice, queue_size=1)
    voice = Voice()
    voice.language = 0
    voice.sentence = arg
    rospy.loginfo('Saying "'+voice.sentence+'" in Japanese.')
    pub.publish(voice)

@respond_to('question (\([^\)]*\)) (\([^\)]*\))')
def question(message, question, answer_candidates):
    question = question.strip('()')
    answer_candidates = answer_candidates.strip('()').split('|')
    user = message.channel._client.users[message.body['user']]
    post_slack_message('Asking "{}" to someone (if any).'.format(question), channel=message.body['channel'])
    tts.say(user['profile']['first_name']+' asks '+question+'. Somebody please answer.')
    sphinx.set_single_rule(answer_candidates+['<NULL>'])
    try:
        ans = sphinx.next_speech(timeout=10.)
        post_slack_message('Someone answered "{}".'.format(ans), channel=message.body['channel'])
    except:
        post_slack_message('Could not get the answer.', channel=message.body['channel'])

@respond_to('roslaunch (.*)')
def roslaunch(message, arg):
    import subprocess
    subprocess.Popen(['/opt/ros/indigo/bin/roslaunch']+arg.split())

@respond_to('rosrun (.*)')
def roslaunch(message, arg):
    import subprocess
    subprocess.Popen(['/opt/ros/indigo/bin/rosrun']+arg.split())

@respond_to('gpsr (.*)')
def gpsr(message, arg):
    say = lambda s: post_slack_message(s, channel=message.body['channel'])
    categories = []
    from gpsr import gpsr_nagoya2017, eegpsr_nagoya2017
    for mod in [gpsr_nagoya2017, eegpsr_nagoya2017]:
        categories += [mod.__getattribute__(name) for name in dir(mod) \
                       if not name.startswith('__')]
    task = None
    for cat in categories:
        parse = cat.GRAMMAR.parse_command(arg)
        if parse is None:
	    continue
        for key, text in parse:
            if key in cat.TASKS:
                say('The command "{}" matches {} in {}.'.format(text, key, cat.__name__))
                task = cat.TASKS[key]
                break
    if not task:
        say('The command "{}" does not match any pattern.'.format(arg))
        return
    class SphinxSim:
        def set_single_rule(self, rule):
            pass
        def set_jsgf(self, jsgf):
            pass
        def next_speech(self, timeout=None):
            key = (message.body['channel'], message.body['user'])
            threads[key] = None
            dt = timeout/20 if timeout is not None else 1.
            rate = rospy.Rate(1/dt)
            count = 0
            while not rospy.is_shutdown():
                val = threads[key]
                if val is not None:
                    threads.pop(key)
                    return val
                count += 1
                if count >= 20:
                    raise Exception('timeout')
                rate.sleep()
        def next_yes_or_no(self, null_weight=500, timeout=None):
            return self.next_speech(timeout=timeout)
        def get_pronunciations(self, word):
            return ['AA']
        def remove_pronunciations(self, word):
            pass
        def add_pronunciations(self, word, pron):
            pass
    sphinx = SphinxSim()
    from common import hokuto, yolo_classes
    sm = task(arena=hokuto, yolo_classes=yolo_classes, robot=robot,
              say_fn=say, sphinx=sphinx)
    sm.userdata.params = task(arg)
    #say(str(sm.userdata.params))
    sm.execute()

@respond_to('pan_camera (.*)')
def pan_camera(message, arg):
    xtion.move(pan=float(arg)*3.1415/180)

@respond_to('tilt_camera (.*)')
def pan_camera(message, arg):
    xtion.move(tilt=float(arg)*3.1415/180)

@respond_to('lift_camera (.*)')
def pan_camera(message, arg):
    xtion.move(lift=float(arg))

@respond_to('panorama')
def panorama_image(message):
    from panorama.srv import MergeImages
    try:
        rospy.wait_for_service('merge_images', timeout=5)
        merge_images = rospy.ServiceProxy('merge_images', MergeImages)
    except:
        #message.reply('Could not find merge_images service. Please run panorama node.')
        post_slack_message('Could not find merge_images service. Please run panorama node.', channel=message.body['channel'])
        return
    pub = rospy.Publisher('talk_request', Voice, queue_size=1)
    voice = Voice()
    voice.language = 0
    voice.sentence = u'パノラマ写真を撮ります'
    pub.publish(voice)
    xtion.move(tilt=0, pan=-0.3, wait=True)
    image1 = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, 1.)
    xtion.move(pan=0., wait=True)
    image2 = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, 1.)
    xtion.move(pan=0.3, wait=True)
    image3 = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image, 1.)

    res = merge_images(image2, [image1, image3])
    panorama = bridge.imgmsg_to_cv2(res.result, 'bgr8')
    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=True) as tf:
        rospy.loginfo('Writing jpeg file '+tf.name+'.')
        try:
            cv2.imwrite(tf.name, panorama)
        except Exception as e:
            rospy.logerr(e)
            post_slack_message('Could not convert the picture ;(', channel=message.body['channel'])
            return
        slacker.files.upload(tf.name, channels=message.body['channel'])

def take_picture():
    try:
        img = rospy.wait_for_message(photo_topic, Image, timeout=5)
        return bridge.imgmsg_to_cv2(img, 'bgr8')
    except Exception as e:
        rospy.logerr(e)
        return

def publish_cv2img(img, topic):
    pub = rospy.Publisher(topic, Image, queue_size=1)
    pub.publish(bridge.cv2_to_imgmsg(img))

@respond_to('photo')
def photo(message):
    cv2_img = take_picture()
    if cv2_img is None:
        post_slack_message('Could not take a picture ;(', channel=message.body['channel'])
    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=True) as tf:
        rospy.loginfo('Writing jpeg file '+tf.name+'.')
        try:
            cv2.imwrite(tf.name, cv2_img)
        except Exception as e:
            rospy.logerr(e)
            post_slack_message('Could not convert the picture ;(', channel=message.body['channel'])
            return
        slacker.files.upload(tf.name, channels=message.body['channel'])

'''
@respond_to('dream')
def dream(message):
    rospy.wait_for_service('deep_dream')
    dream = rospy.ServiceProxy('deep_dream', DeepDream)
    res = dream(photo_topic)
    cv2_img = bridge.imgmsg_to_cv2(res.dream)
    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=True) as tf:
        rospy.loginfo('Writing jpeg file '+tf.name+'.')
        try:
            cv2.imwrite(tf.name, cv2_img)
        except Exception as e:
            rospy.logerr(e)
            post_slack_message('Could not convert the picture ;(')
            return
        slacker.files.upload(tf.name, channels=message.body['channel'])
'''

@default_reply
def publish_message(message):
    #post_slack_message(str(threads), channel=message.body['channel'])
    key = (message.body['channel'],message.body['user'])
    if key in threads:
        threads[key] = message.body['text']
    else:
        global pub
        pub.publish(to_ros_msg(message.body))

def talk_cb(msg):
    if isinstance(msg, TalkRequestActionGoal):
        msg = msg.goal.data
    post_slack_message('I said "' + msg.sentence+'".')

def listen_cb(msg):
    post_slack_message('I heard "' + msg.data+'".')

def sigint_handler(signum, frame):
    rospy.signal_shutdown('Manual shutdown')
    time.sleep(1)
    quit()
signal.signal(signal.SIGINT, sigint_handler)

#rospy.init_node('slack_message')
rospy.on_shutdown(lambda: os._exit(0))

pub = rospy.Publisher('slack', Message, queue_size=10)
tf_buffer = robot._get_tf2_buffer()
xtion = HSRB_Xtion(tf_buffer=tf_buffer)
tts = DefaultTTS()
sub_talk = rospy.Subscriber(talk_topic, Voice, talk_cb)
sub_talk_goal = rospy.Subscriber(talk_goal, TalkRequestActionGoal, talk_cb)
sub_listen = rospy.Subscriber(listen_topic, String, listen_cb)
rospy.loginfo('Waiting for pocket sphinx...')
sphinx = PocketSphinxClient()
rospy.loginfo('Pocket sphinx is ready.')
bot = Bot()
bot.run()

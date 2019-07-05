# raspberrypi-slackbot

## Requirements
- OpenCV
```
sudo apt-get install libopencv-dev python-opencv
```
- Slacker API
```
pip install slacker slackbot
```

## Setup
1. Create a bot account on your slack group and get its API key.
1. Clone this repository.
```
git clone https://github.com/hirokiyokoyama/raspberrypi-slackbot
```
1. Put the API key into slackbot_settings.py.
```
cd raspberrypi-slackbot
vi slackbot_settings.py

API_TOKEN = '[your bot's API key]'
```

## How to run
1. Launch the program on your raspberrypi.
```
cd raspberrypi-slackbot
./main.py
```
1. Talk to the bot on Slack.

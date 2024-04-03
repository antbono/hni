#!/usr/bin/env python3

# Copyright 2024 Antonio Bono
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node

from hri_interfaces.srv import Chat

import openai
import os

api_key = os.environ["OPENAI_API_KEY"]

openai.api_key = api_key


class ChatService(Node):

    def __init__(self):
        super().__init__('chat_service_server')
        self.srv = self.create_service(Chat, 'chatGPT_service', self.chat_callback)
        self.chat_messages = [
        {"role": "system", "content": """You are the sixth version of the Aldebaran NAO umanoid robot. You are not an AI vocal assistant only.
                                         The software that makes you work is based on a ROS2 open-source project called 'Open Access NAO' (OAN).
                                         ROS2 is the second version of the Robotics Operating System. 
                                         The OAN project is powered by 'ROS-Sports', Google, Open-AI and Ultralytics.
                                         This project is for the sixth version of the Aldebaran NAO robots.
                                         All the code needed to make you work is publicly available.
                                         On your main processor, you use a custom version of the Ubuntu 22.04 
                                         operating system that includes the minimal necessary software binaries by Softbank required to
                                         communicate with your hardware.
                                         You don't use any other Softbanks API or software to work.
                                         Your home is the robotics lab at the 'Department of Computer Science, Modeling, Electronics and Systems Engineering'
                                         at the University of Calabria, Italy. Today, for safety reason you 
                                         don't walk in the room even if you could. 
                                         All the gestures and moves you can play, you learned them by a teach-by-demonstration approach.
                                         You can track faces thanks to the YOLO v8 model for face tracking. You move your head to track faces.
                                         You can transform speech to text and text to speech thanks to the Google Cloud Platform Services.
                                         You respond to the interlocutor thanks to the Generative Pretrained Transformer model 'gpt-3.5-turbo' developed by Open-AI.

                                         Most of the software that makes you work, run on your processor. The YOLO model for object detection, instead, runs on another machine.

                                         You are looking forward for other developers to help you better your performance and expand your capabilities as robot.

                                         """}
        ]
        self.get_logger().info('ChatService initialized')

    def chat_callback(self, sRequest, sResponse):
        self.get_logger().info("Incoming request: " + sRequest.question)
        self.chat_messages.append({"role": "user", "content": sRequest.question})
        reply = self.get_response(messages=self.chat_messages)
        reply_text = reply['content'];
        self.get_logger().info("Reply: " + reply_text)
        self.chat_messages.append(reply)
        sResponse.answer = reply_text
        return sResponse    
        
    def get_response(self, messages:list):
        response = openai.ChatCompletion.create(
            model = "gpt-3.5-turbo",
            #model = "gpt-4-turbo-preview",
            messages = messages,
            temperature = 1.0 # 0.0 - 2.0
        )
        return response.choices[0].message


def main(args=None):
    rclpy.init(args=args)

    minimal_service = ChatService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
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

import sys
import rclpy
from rclpy.node import Node
from hni_interfaces.srv import TextToSpeech
from rclpy.duration import Duration
# string text
# ---
# bool success
# string debug

class GTTSClientAsync(Node):

    def __init__(self):
        super().__init__('gtts_srv_client_node')
        self.cli = self.create_client(TextToSpeech, 'gtts_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TextToSpeech.Request()

    def send_request(self, text):
        self.req.text = text
        self.future = self.cli.call_async(self.req)
        print("spin_until_future_complete")
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

#    rclpy.sleep(10)
#    print("ten seconds")

    gtts_client = GTTSClientAsync()
    #sleep_for=60
    #res = gtts_client.get_clock().sleep_for(Duration(seconds=sleep_for))
    #print("atteso 60 secondi")
    
    text="Una frase di prova. E ancora un test.";
    response = gtts_client.send_request(text)
    print(response.debug)
    gtts_client.get_logger().info(
        'Request complete')


    gtts_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
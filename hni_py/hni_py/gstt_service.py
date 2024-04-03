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

from google.cloud import speech
#from google.cloud import speech_v1p1beta1 as speech
from google.rpc import code_pb2
from google.api_core.exceptions import GoogleAPICallError
from google.api_core.exceptions import DeadlineExceeded
from google.protobuf.duration_pb2 import Duration

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from hni_py.transcribe_streaming_mic import MicrophoneStream
from std_srvs.srv import SetBool

# bool data # e.g. for hardware enabling / disabling
# ---
# bool success   # indicate successful run of triggered serviceyapf
# string message # informational, e.g. for error messages

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class GSTTService(Node):
    def __init__(self):
        super().__init__("gstt_srv_node")

        # Create a Duration object
        duration1 = Duration()
        duration1.seconds=40
        duration2 = Duration()
        duration2.seconds=50

        language_code = "en-US" # a BCP-47 language tag
        self.client = speech.SpeechClient()
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code,
            #model="latest_short",
            model="latest_long"
        )
        self.timeout = speech.StreamingRecognitionConfig.VoiceActivityTimeout(
            speech_start_timeout=duration1,
            speech_end_timeout=duration2
        )

        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config,
            #single_utterance=True,
            #interim_results=True,
            enable_voice_activity_events=True, voice_activity_timeout=self.timeout
        )

        self.srv = self.create_service(SetBool, "gstt_service", self.gstt_callback)

        #self.stream = MicrophoneStream(RATE, CHUNK)



        self.get_logger().info('GSTTService initializedaa')


    
    def gstt_callback(self, sRequest, sResponse):
        
        self.get_logger().info('GSTTService Incoming request')

        if sRequest.data == True:

            try:
                sResponse.success=False

                stream = MicrophoneStream(RATE, CHUNK)
                stream = stream.__enter__()
                audio_generator = stream.generator()
                requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator
                )

                self.get_logger().info('requests created')
                responses_iterator = self.client.streaming_recognize(self.streaming_config, requests)
                self.get_logger().info('responses created')
                
                start_time = time.time()
                self.get_logger().info('start_time')
                timeout_seconds = 30
                final_transcript_received = False

                for response in responses_iterator:         #BLOCKING!
                    self.get_logger().info('responses loop')
                    if time.time() - start_time > timeout_seconds:
                        self.get_logger().error("Timeout: No final result after %d seconds." % timeout_seconds)
                        sResponse.success = False
                        sResponse.message = "timeout"
                        break  # Exit the loop if we've reached the timeout without a final result
                    self.get_logger().info('timeout checked')    

                    # Check if there are any results in this response
                    if not response.results:
                        continue
                    self.get_logger().info('results checked')     
                        
                    # The first result is the most relevant for single utterance mode
                    result = response.results[0]

                    # Check if the result is final
                    if result.is_final:
                        final_transcript_received = True
                        # Extract the top alternative of the final result
                        top_transcript = result.alternatives[0].transcript
                        self.get_logger().info(f"Final transcript: {top_transcript}")
                        sResponse.success = True
                        sResponse.message = top_transcript
                        break  # Exit the loop since we've received a final transcript
                    self.get_logger().info('final result checked')

                    if not final_transcript_received:
                        self.get_logger().error("No final transcript received.")
                        sResponse.success = False
                        sResponse.message = "No final transcript received."

                self.get_logger().debug('GSTTService complete request')
                return sResponse

            except Exception as e:
                self.get_logger().error(f"Error during speech recognition: {e}")
                sResponse.success = False
                return sResponse

            finally:
                stream.__exit__(stream, stream, stream)



            #After stream close
            #sResponse.success = False
            #sResponse.message = "No final result was obtained."
            #self.get_logger().info('No final result was obtained.')
            #return sResponse

        else:
            sResponse.success = False
            sResponse.message = "The service must be called with 'True' to start recognition."
            return sResponse
            

    def __retrieve_text(self, responses):
            """Iterates through server responses and prints them.

            The responses passed is a generator that will block until a response
            is provided by the server.

            Each response may contain multiple results, and each result may contain
            multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
            print only the transcription for the top alternative of the top result.

            In this case, responses are provided for interim results as well. If the
            response is an interim one, print a line feed at the end of it, to allow
            the next result to overwrite it, until the response is a final one. For the
            final one, print a newline to preserve the finalized transcription.
            """

            num_loop = 0
            num_not_results = 0
            num_not_alternatives = 0
            num_not_final = 0

            try:
                for response in responses:
                    num_loop = num_loop + 1
                    self.get_logger().info('num loop %d' % num_loop)
                    if not response.results:
                        num_not_results = num_not_results + 1
                        self.get_logger().info('no results %d' % num_not_results)
                        continue

                    # The `results` list is consecutive. For streaming, we only care about
                    # the first result being considered, since once it's `is_final`, it
                    # moves on to considering the next utterance.
                    self.get_logger().info('try a result')    
                    result = response.results[0]
                    if not result.alternatives:
                        num_not_alternatives = num_not_alternatives + 1
                        self.get_logger().debug('no alternative %d' % num_not_alternatives)
                        continue

                    self.get_logger().info('alternative present')    
                        
                    # Display the transcription of the top alternative.
                    transcript = result.alternatives[0].transcript

                    # Display interim results, but with a carriage return at the end of the
                    # line, so subsequent lines will overwrite them.
                    #
                    # If the previous result was longer than this one, we need to print
                    # some extra spaces to overwrite the previous result
                    #overwrite_chars = " " * (num_chars_printed - len(transcript))
                    

                    if not result.is_final:
                        #sys.stdout.write(transcript + overwrite_chars + "\r")
                        #self.get_logger().info(transcript + overwrite_chars + "\r")
                        num_not_final = num_not_final + 1
                        self.get_logger().info('not final %d' % num_not_final)

                    else:
                        self.get_logger().info(transcript)
                        num_chars_printed = 0
                        return transcript

            except GoogleAPICallError as e:
                if e.code == code_pb2.PERMISSION_DENIED:
                    self.get_logger().error("Permission denied error.")
                elif e.code == code_pb2.NOT_FOUND:
                    self.get_logger().error("Resource not found.")
                else:
                    self.get_logger().error(f"An error occurred: {e}")
            return ""    
    

def main():
    rclpy.init()

    stt_service = GSTTService()

    rclpy.spin(stt_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

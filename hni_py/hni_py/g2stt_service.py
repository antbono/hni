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

import os

from time import sleep

from google.api_core.exceptions import GoogleAPICallError
from google.api_core.exceptions import DeadlineExceeded
from google.protobuf.duration_pb2 import Duration

from google.cloud.speech_v2 import SpeechClient
from google.cloud.speech_v2.types import cloud_speech
from google.protobuf import duration_pb2  # type: ignore

import rclpy
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
PROJECT_ID = os.environ["GCP_OAN_ID"]

class G2STTService(Node):
    def __init__(self):
        super().__init__("gstt_srv_node")

        #language_code = "en-US" # a BCP-47 language tag

         # Instantiates a client
        self.client = SpeechClient()

        recognition_config = cloud_speech.RecognitionConfig(
            auto_decoding_config=cloud_speech.AutoDetectDecodingConfig(),
            language_codes=["en-US"],
            model="long",
        )

        # Sets the flag to enable voice activity events and timeout
        speech_start_timeout = duration_pb2.Duration(seconds=5)
        speech_end_timeout = duration_pb2.Duration(seconds=20)
        voice_activity_timeout = (
            cloud_speech.StreamingRecognitionFeatures.VoiceActivityTimeout(
                speech_start_timeout=speech_start_timeout,
                speech_end_timeout=speech_end_timeout,
            )
        )
        streaming_features = cloud_speech.StreamingRecognitionFeatures(
            enable_voice_activity_events=True, voice_activity_timeout=voice_activity_timeout
        )

        streaming_config = cloud_speech.StreamingRecognitionConfig(
            config=recognition_config, streaming_features=streaming_features
        )

        self.config_request = cloud_speech.StreamingRecognizeRequest(
            recognizer=f"projects/{PROJECT_ID}/locations/global/recognizers/_",
            streaming_config=streaming_config,
        )    


        self.srv = self.create_service(SetBool, "gstt_service", self.gstt_callback)

        #self.stream = MicrophoneStream(RATE, CHUNK)
        loop_rate=2

        self._loop_rate = self.create_rate(loop_rate, self.get_clock())

        self.get_logger().info('G2STTService initialized')

    
    def gstt_callback(self, sRequest, sResponse):
        
        self.get_logger().debug('G2STTService Incoming request')

        if sRequest.data == True:

            try:
                sResponse.success = False

                stream = MicrophoneStream(RATE, CHUNK)
                stream = stream.__enter__()
                audio_generator = stream.generator()

                audio_requests = (
                    cloud_speech.StreamingRecognizeRequest(audio=content) for content in audio_generator
                )
                self.get_logger().info('requests created')

                #def requests(config: cloud_speech.RecognitionConfig, audio: list) -> list:
                #    yield config
                #    for message in audio:
                #        self._loop_rate.sleep()
                #        yield message

                # Transcribes the audio into text
                self.get_logger().info('creating responses')
                responses_iterator = self.client.streaming_recognize( 
                    #requests=requests(self.config_request, audio_requests)
                    config=self.config_request, requests=audio_requests
                )
                self.get_logger().info('responses created')
                
                final_transcript_received = False

                #responses = []
                for response in responses_iterator:         #BLOCKING!
                    self.get_logger().info('responses loop')

                    #responses.append(response)
                    if (
                        response.speech_event_type
                        == cloud_speech.StreamingRecognizeResponse.SpeechEventType.SPEECH_ACTIVITY_BEGIN
                    ):
                        self.get_logger().warn('Speech Started.')
                    if (
                        response.speech_event_type
                        == cloud_speech.StreamingRecognizeResponse.SpeechEventType.SPEECH_ACTIVITY_END
                    ):
                        self.get_logger().warn('Speech Ended.')

                    # Check if there are any results in this response
                    if not response.results:
                        continue    
    
                     # The first result is the most relevant 
                    result = response.results[0]

                    if result.is_final:
                        final_transcript_received = True
                        top_transcript = result.alternatives[0].transcript
                        self.get_logger().info(f"Final transcript: {top_transcript}")
                        sResponse.success = True
                        sResponse.message = top_transcript
                        break  # Exit the loop since we've received a final transcript

                    if not final_transcript_received:
                        self.get_logger().error("No final transcript received.")
                        sResponse.success = False
                        sResponse.message = "No final transcript received."

                self.get_logger().debug('G2STTService complete request')
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

    stt_service = G2STTService()

    rclpy.spin(stt_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

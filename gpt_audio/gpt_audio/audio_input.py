#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This script defines an audio input node for a robot's speech-related functionality.
# The AudioInput class utilizes AWS services to record user audio input,
# perform transcription with AWS Transcribe, and publish the transcribed text back to ROS2.
# The input volume can be re-scaled for robots with low output volume such as Mini Pupper v2.
# It also supports continuous audio input and real-time transcription by checking the GPT status
# and executing the audio input and transcription process accordingly.
# The transcription results are published as ROS2 String messages to the "gpt_text_input_original" topic.
# Make sure to configure the necessary AWS credentials and settings in the provided config file.
#
# Author: Herman Ye

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# AWS ASR related
import boto3
import datetime

# Audio recording related
import sounddevice as sd
from scipy.io.wavfile import write

# GPT related
from gpt_status.gpt_param_server import GPTStatus, GPTStatusOperation
from gpt_status.gpt_config import GPTConfig

# Other libraries
import time
import json
import requests
import os

config = GPTConfig()


class AudioInput(Node):
    def __init__(self):
        super().__init__("audio_input", namespace="gpt")
        # Publisher
        self.publisher = self.create_publisher(
            String, "gpt_text_input_original", 10
        )
        # Timer
        self.create_timer(1, self.run_audio_input_callback)
        # AWS service initialization
        self.aws_audio_file = "/tmp/user_audio_input.flac"
        self.aws_access_key_id = config.aws_access_key_id
        self.aws_secret_access_key = config.aws_secret_access_key
        self.aws_region_name = config.aws_region_name
        self.aws_session = boto3.Session(
            aws_access_key_id=self.aws_access_key_id,
            aws_secret_access_key=self.aws_secret_access_key,
            region_name=self.aws_region_name,
        )

        # Set the speaker volume to maximum for mini pupper v2
        # If you are using a different robot, please comment out the lines
        self.volume_gain_multiplier = config.volume_gain_multiplier
        self.declare_parameter("mini_pupper", False)  # default is False
        self.is_mini_pupper = self.get_parameter("mini_pupper").value
        if self.is_mini_pupper:
            self.get_logger().info("Mini pupper v2 mode is enabled.")
            os.system("amixer -c 0 sset 'Headphone' 100%")
            self.volume_gain_multiplier = 30
            self.get_logger().info(
                "Volume gain multiplier is set to 30 for mini pupper v2."
            )
        # GPT status initialization
        self.gpt_operation = GPTStatusOperation()
        # Audio input initialization status for console output
        self.get_logger().info("Audio input successfully initialized.")

    def run_audio_input_callback(self):
        gpt_current_status_value = self.gpt_operation.get_gpt_status_value()
        # Check if GPT status is WAITING_USER_INPUT
        if gpt_current_status_value == GPTStatus.WAITING_USER_INPUT.name:
            self.run_audio_input()

    def run_audio_input(self):
        # Recording settings
        duration = config.duration  # Audio recording duration, in seconds
        sample_rate = config.sample_rate  # Sample rate

        # For Mangdang mini pupper v2 quadruped robot, the volume is too low
        # so we need to increase the volume by 30x

        # AWS S3 settings
        bucket_name = config.bucket_name
        audio_file_key = "gpt_audio.flac"  # Name of the audio file in S3
        transcribe_job_name = f'my-transcribe-job-{datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}'
        # Name the conversion task based on time to ensure uniqueness
        # Path of the audio file in S3
        transcribe_job_uri = f"s3://{bucket_name}/{audio_file_key}"

        # Step 1: Record audio
        self.get_logger().info("Starting audio recording...")
        audio_data = sd.rec(
            int(duration * sample_rate), samplerate=sample_rate, channels=1
        )
        sd.wait()  # Wait until recording is finished

        # Step 2: Increase the volume by a multiplier
        audio_data *= self.volume_gain_multiplier

        # Step 3: Save audio to file
        write(self.aws_audio_file, sample_rate, audio_data)
        self.get_logger().info("Audio recording complete!")

        # Set GPT status to SPEECH_TO_TEXT_PROCESSING
        self.gpt_operation.set_gpt_status_value(
            GPTStatus.SPEECH_TO_TEXT_PROCESSING.name
        )
        # Step 4: Upload audio to AWS S3
        s3 = self.aws_session.client("s3")
        self.get_logger().info("Uploading audio to AWS S3...")
        with open(self.aws_audio_file, "rb") as f:
            s3.upload_fileobj(
                Fileobj=f, Bucket=bucket_name, Key=audio_file_key
            )
        self.get_logger().info("Audio upload complete!")

        # Step 5: Convert audio to text
        transcribe = self.aws_session.client("transcribe")
        self.get_logger().info("Starting audio to text conversion...")
        transcribe.start_transcription_job(
            TranscriptionJobName=transcribe_job_name,
            LanguageCode=config.aws_transcription_language,
            Media={"MediaFileUri": transcribe_job_uri},
        )

        # Step 6: Wait until the conversion is complete
        while True:
            status = transcribe.get_transcription_job(
                TranscriptionJobName=transcribe_job_name
            )
            if status["TranscriptionJob"]["TranscriptionJobStatus"] in [
                "COMPLETED",
                "FAILED",
            ]:
                break
            time.sleep(1)
            self.get_logger().info("Converting...")

        # Step 7: Get the transcribed text
        if status["TranscriptionJob"]["TranscriptionJobStatus"] == "COMPLETED":
            transcript_file_url = status["TranscriptionJob"]["Transcript"][
                "TranscriptFileUri"
            ]
            response = requests.get(transcript_file_url)
            transcript_data = json.loads(response.text)
            transcript_text = transcript_data["results"]["transcripts"][0][
                "transcript"
            ]
            self.get_logger().info("Audio to text conversion complete!")
            # Step 8: Publish the transcribed text to ROS2
            if transcript_text == "":  # Empty input
                # Set GPT status to WAITING_USER_INPUT
                self.gpt_operation.set_gpt_status_value(
                    GPTStatus.WAITING_USER_INPUT.name
                )
                self.get_logger().info(
                    "Empty input, waiting for user input..."
                )
            else:
                msg = String()
                msg.data = transcript_text
                self.publisher.publish(msg)
                self.get_logger().info(
                    "Audio Input Node publishing: \n'%s'" % msg.data
                )

            # Step 9: Delete the temporary audio file from AWS S3
            s3.delete_object(Bucket=bucket_name, Key=audio_file_key)

        else:
            self.get_logger().error(
                f"Failed to transcribe audio: {status['TranscriptionJob']['FailureReason']}"
            )


def main(args=None):
    rclpy.init(args=args)

    audio_input = AudioInput()

    rclpy.spin(audio_input)

    audio_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

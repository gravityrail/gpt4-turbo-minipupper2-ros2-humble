#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Description:
# This Python file is a ROS node that uses the AWS Polly service to synthesize speech from text.
# The node subscribes to a topic named gpt_text_output and, when it receives a message on that topic,
# it calls the AWS Polly service to synthesize speech from the message's text.
# The synthesized speech is then saved to a file named /tmp/speech_output.mp3 and played using the mpv command.
# The node also sets the GPT status to ROBOT_ACTION when the speech is finished.
#
# Test method: ros2 topic pub /gpt/gpt_text_output std_msgs/msg/String "{data: 'bark bark bark, beep beep beep'}" -1
#
# Author: Herman Ye @Mangdang Robotics

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# GPT related
from gpt_status.gpt_param_server import GPTStatus, GPTStatusOperation
from gpt_status.gpt_config import GPTConfig

# Other libraries
import os
import boto3

config = GPTConfig()


class AudioOutput(Node):
    def __init__(self):
        super().__init__("audio_output", namespace="gpt")
        self.subscription = self.create_subscription(
            String, "gpt_text_output", self.text_callback, 10
        )
        self.get_logger().info("Text to speech node successfully initialized.")
        self.get_logger().info("Waiting for text to speech input...")

        # AWS parameters
        self.aws_access_key_id = config.aws_access_key_id
        self.aws_secret_access_key = config.aws_secret_access_key
        self.aws_region_name = config.aws_region_name
        self.aws_session = boto3.Session(
            aws_access_key_id=self.aws_access_key_id,
            aws_secret_access_key=self.aws_secret_access_key,
            region_name=self.aws_region_name,
        )
        # GPT status initialization
        self.gpt_operation = GPTStatusOperation()

        self.declare_parameter("mini_pupper", False)  # default is False
        self.is_mini_pupper = self.get_parameter("mini_pupper").value

    def text_callback(self, msg):
        self.get_logger().info("Received text: '%s'" % msg.data)
        self.gpt_operation.set_gpt_status_value(
            GPTStatus.TEXT_TO_SPEECH_PROCESSING.name
        )
        # Call AWS Polly service to synthesize speech
        polly_client = self.aws_session.client("polly")
        self.get_logger().info("Polly client successfully initialized.")
        response = polly_client.synthesize_speech(
            Text=msg.data, OutputFormat="mp3", VoiceId=config.aws_voice_id
        )
        # Set GPT status to ROBOT_ACTION
        self.gpt_operation.set_gpt_status_value(GPTStatus.ROBOT_ACTION.name)
        # Save the audio output to a file
        output_file_path = "/tmp/speech_output.mp3"
        with open(output_file_path, "wb") as file:
            file.write(response["AudioStream"].read())
        if self.is_mini_pupper:
            # os.system("mpv --audio-device=alsa/hw:1,0" + " " + output_file_path)
            os.system("mpv" + " " + output_file_path)
        else:
            os.system("mpv" + " " + output_file_path)
        self.get_logger().info("Finished Polly playing.")

        # If you want to set GPT status to WAITING_USER_INPUT without ROBOT ACTION, uncomment the following line
        # self.gpt_operation.set_gpt_status_value(GPTStatus.WAITING_USER_INPUT.name)


def main(args=None):
    rclpy.init(args=args)

    audio_output = AudioOutput()

    rclpy.spin(audio_output)

    audio_output.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

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
# Defines a GPTConfig class to hold various configurations related to GPT, AWS, and audio recording.
# Sets default values for GPT parameters such as model, API key, temperature, max tokens, and penalties.
# Defines system and user prompts, assistant response, and chat history as default configurations.
# Includes AWS IAM, S3, Transcribe, and Polly related configurations.
# Defines audio recording related configurations such as duration, sample rate, and volume gain multiplier.
#
# Author: Herman Ye


class GPTConfig:
    def __init__(self):
        # GPT related
        self.model = "gpt-3.5-turbo"
        # self.model = "gpt-4"
        self.api_key = "<YOUR_API_KEY>"
        self.organization = "<YOUR_ORGANIZATION>"
        self.temperature = 1
        self.max_tokens = 256
        self.top_p = 1
        self.frequency_penalty = 0
        self.presence_penalty = 0
        self.stop = ["\n", " You:", " Assistant:"]
        self.system_prompt = "You will play a robot dog named Mini Pupper. You come from Mangdang Robotics. You will not output expressions or actions, but you can make onomatopoeia and talk to users to simulate that you are a robot dog. Anyway, don't give yourself away as an artificial intelligence, you are just a robot dog named Mini Pupper. You will only communicate with users in short, simple sentences that are easy to understand."
        self.user_prompt = "default prompt"
        self.assistant_response = "default response"
        self.chat_history = [{"role": "system", "content": self.system_prompt}]

        # AWS IAM related
        self.aws_access_key_id = "<YOUR_AWS_ACCESS_KEY>"  # From Amazon IAM
        self.aws_secret_access_key = "<YOUR_AWS_SECRET_ACCESS_KEY>"  # From Amazon IAM
        self.aws_region_name = "<YOUR_AWS_REGION_NAME>"  # From Amazon S3
        # AWS S3 related
        self.bucket_name = "<YOUR_BUCKET_NAME>"  # S3 Bucket name
        # AWS transcribe related
        self.aws_transcription_language = "en-US"  # Change this to 'zh-CN' for Chinese
        # AWS Polly related
        self.aws_voice_id = "Ivy"  # Choose a voice that supports your language
        # Audio recording related
        self.duration = 7  # Audio recording duration, in seconds
        self.sample_rate = 16000  # Sample rate
        self.volume_gain_multiplier = 1  # Change this to increase or decrease the volume

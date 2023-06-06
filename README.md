[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html) &nbsp; [![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/) &nbsp; [![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/gpt4_ros2/blob/main/LICENSE) &nbsp;

# gpt4_ros2

gpt4_ros2 project is a ROS2 Humble interface designed to empower your robots with advanced voice interaction and motion control capabilities. This package enables you to leverage LLM-based features, such as GPT-4 & ChatGPT, to enhance the functionality of their robotic applications within the ROS2 ecosystem. With an easy-to-use installation and customization process, gpt4_ros2 provides a dynamic solution for creating engaging and interactive experiences with any robot. 

# Installation

## One-click Installation

To install with one command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/install.sh https://raw.githubusercontent.com/mangdangroboticsclub/gpt4_ros2/main/install.sh && sudo chmod +x $HOME/install.sh && bash $HOME/install.sh && rm $HOME/install.sh
```
After the one-click Installation, `demo 1 Simple robot GPT call on the PC side` will run automatically, if you want to run other demos, please modify the configuration file according to Step4 of Manual Installation
## Manual Installation

If you want to install manually, follow the steps below.

### Step 1: Clone the repo

```bash
cd <your_ws>/src
git clone https://github.com/mangdangroboticsclub/gpt4_ros2.git
```

### Step 2: Install dependencies

```bash
cd <your_ws>/src/gpt4_ros2
. dependencies_install.sh # Install dependencies
```

### Step 3: Build the repo

```bash
cd <your_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Step 4: Configuration

To use the gpt4_ros2 package, follow these steps:

#### 4.1 Set up AWS

##### 4.1.1 Create an S3 bucket
1. Create an AWS account.
2. Go to the AWS Console Amazon S3 page.
3. Click `Create bucket`.
4. Enter a `Bucket name` and choose an `AWS Region`.
5. Click `Create bucket`.

##### 4.1.2 Set up an IAM user
1. Go to the AWS Console Amazon IAM page.
2. Click `Users` under the `IAM resources` section.
3. Click `Add user`.
4. Enter a `User name`.
5. Under `Set permissions`, click `Attach existing policies directly` and search for the following policies:
   - `AmazonPollyFullAccess`
   - `AmazonTranscribeFullAccess`
   - `AmazonS3FullAccess`
6. Add the selected policies to the user.
7. Click `Next`, review the `Permissions summary` and any other information.
8. Click `Create user`.

#### 4.2 Set up OpenAI API
1. Create an account on [OpenAI](https://platform.openai.com).
2. Click on the user icon in the upper-right corner.
3. Click `View API keys`.
4. Click `Create new secret key`.
5. Enter a `name` and click `Create secret key`.
6. Copy your secret key and save it securely.

#### 4.3 Configure and build the package
1. Navigate to `<your_ws>/src/src/gpt4_ros2/gpt_status/gpt_status/gpt_config.py`.
```bash
cd <your_ws>/src/src/gpt4_ros2/gpt_status/gpt_status
```
2. Set your desired configurations, such as the GPT-4 or GPT-3.5-turbo model, system_prompt, and other attributes. Fill in the relevant configuration details for AWS and OpenAI that you obtained earlier.
```bash
sudo nano gpt_config.py
```

#### 4.4 Modify the gpt_robot package code [optional]
If you wish to use GPT for your own robots, modify the contents of the gpt_robot package, which configures physical or virtual robots.

We encourage you to customize the GPTConfig class to tailor the functionality of this ROS2 wrapper for GPT-4 and ChatGPT (GPT-3.5) according to your specific needs. To do this, simply modify the values in the code snippet to suit your requirements:

GPT-4 is currently in a limited beta and only accessible to those who have been granted access. Please join the [waitlist](https://openai.com/waitlist/gpt-4-api) to get access when capacity is available.

Feel free to adjust the parameters, such as temperature, max_tokens, and top_p, to influence the behavior of the GPT model. You can also customize the system_prompt and user_prompt strings to create unique and engaging interactions with your robot.

By personalizing these settings, you can create a one-of-a-kind experience tailored to your specific robotic application. Enjoy experimenting and discovering new possibilities!

# Usage

## Demo 1: Simple robot GPT call on the PC side

If you want to simply try this service, configure your OpenAI API and system_prompt in `gpt_status/gpt_config.py`. Then, try:

```bash
# Terminal 1
ros2 run gpt_main gpt_ros2_server
```

```bash
# Terminal 2
ros2 run gpt_main gpt_ros2_client
```

## Demo 2: Using GPT service on a quadruped robot dog

The gpt4_ros2 package supports Mangdang Robot's latest `Mini Pupper V2` quadruped robot dog. If you want to learn more about the Mini Pupper four-legged robot dog, please check [Mangdang's GitHub home page](https://github.com/mangdangroboticsclub?tab=repositories).

After configuring everything, run:
```bash
# Terminal 1 Bringup mini pupper
ros2 launch mini_pupper_bringup bringup.launch.py
```
```bash
# Terminal 2 Bringup GPT
ros2 launch gpt_bringup gpt_bringup_launch.py mini_pupper:=True
```

## Demo 3: Using GPT service on your own robot

You can run this project on your robot or PC. If you want to set GPT-related content for the robot, modify the gpt_robot package and run the following command. If you don’t have a robot, you can also run it directly on your computer:

```bash
ros2 launch gpt_bringup gpt_bringup_launch.py
```

If you find this project useful, please consider giving it a ⭐️ star on GitHub! Your support helps us improve the project and encourages further development. Don't forget to also share it with your friends and colleagues who might it beneficial. Thank you for your support! 
# Contributing
Contributions are welcome! Please read the [contributing guidelines](CONTRIBUTING.md) before submitting a pull request.



# License
This project is licensed under the Apache-2.0 License. See [LICENSE](LICENSE) for more information.
```
Copyright 2023 Mangdang
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.                             
```
# Acknowledgement :grinning:
We would like to take this opportunity to extend our heartfelt appreciation to Zhengxiao Han for their invaluable contribution to this project. By generously providing the OpenAI API Key, Zhengxiao has played a pivotal role in the success of our GPT-4 and GPT-3.5(Chat GPT) interface for ROS2 Humble.

Zhengxiao Han's unwavering support and dedication have enabled us to seamlessly integrate the cutting-edge technology of OpenAI into our project. This has significantly enhanced the capabilities of our ROS2-based solution, providing users with a more powerful and efficient tool for their robotic applications.

Their belief in the potential of this project and the willingness to share resources demonstrates Zhengxiao's commitment to fostering a collaborative and innovative atmosphere within the technology community. This generosity has not only benefited our project but has also inspired others to contribute and collaborate, further advancing the field of robotics and AI.

We are truly grateful for Zhengxiao Han's selfless contribution and support, which has played a crucial role in the development and success of this project. Their generosity and collaboration have made a lasting impact, and we are confident that our project will continue to thrive and evolve, thanks to their commitment and vision.

Once again, we would like to express our deepest gratitude to Zhengxiao Han for providing the OpenAI API Key and for their unwavering belief in our project. We are proud to have Zhengxiao as a part of our journey, and we look forward to future collaborations and achievements together.

(The above acknowledgments were generated by GPT-4)

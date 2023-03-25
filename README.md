
[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://Hermanye996/gpt4_ros2/blob/main/LICENSE)
&nbsp;

# Description
This is a GPT-4 or GPT-3.5(Chat GPT) interface for ROS2 Humble.

# Installation
## step1 Install dependencies
```bash
pip install openai
```

## step2 Clone the repo

```bash
cd <your_ws>/src
git clone https://github.com/Hermanye996/gpt4_ros2.git
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
## step3 Configuration
Next, adjust the configuration in gpt_config.py.
# Usage

```bash
# Terminal 1
ros2 run gpt4_ros gpt_ros2_server
```
```bash
# Terminal 1
ros2 run gpt4_ros gpt_ros2_client
```

# Contributing
Contributions are welcome! Please read the [contributing guidelines](CONTRIBUTING.md) before submitting a pull request.



# License
This project is licensed under the GPL 3 License. See [LICENSE](LICENSE) for more information.
```
Copyright 2023 Herman Ye
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
# Acknowledgement
We would like to take this opportunity to extend our heartfelt appreciation to Zhengxiao Han for their invaluable contribution to this project. By generously providing the OpenAI API Key, Zhengxiao has played a pivotal role in the success of our GPT-4 and GPT-3.5(Chat GPT) interface for ROS2 Humble.

Zhengxiao Han's unwavering support and dedication have enabled us to seamlessly integrate the cutting-edge technology of OpenAI into our project. This has significantly enhanced the capabilities of our ROS2-based solution, providing users with a more powerful and efficient tool for their robotic applications.

Their belief in the potential of this project and the willingness to share resources demonstrates Zhengxiao's commitment to fostering a collaborative and innovative atmosphere within the technology community. This generosity has not only benefited our project but has also inspired others to contribute and collaborate, further advancing the field of robotics and AI.

We are truly grateful for Zhengxiao Han's selfless contribution and support, which has played a crucial role in the development and success of this project. Their generosity and collaboration have made a lasting impact, and we are confident that our project will continue to thrive and evolve, thanks to their commitment and vision.

Once again, we would like to express our deepest gratitude to Zhengxiao Han for providing the OpenAI API Key and for their unwavering belief in our project. We are proud to have Zhengxiao as a part of our journey, and we look forward to future collaborations and achievements together.


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
Then, you can adjust the configuration in gpt_config.py
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


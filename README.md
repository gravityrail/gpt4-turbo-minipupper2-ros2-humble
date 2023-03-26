
[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://Hermanye996/gpt4_ros2/blob/main/LICENSE)
&nbsp;

# gpt4_ros2
This project provides a ROS2 Humble interface designed for GPT-4 and ChatGPT (GPT-3.5), featuring built-in ROS2 client and server functionality. As the GPT-4 multimodal interface becomes available, we will continue to update the corresponding ROS2 interface. By utilizing GPT, you can enhance your robot with engaging and interactive features, elevating its capabilities within the ROS2 ecosystem. This ROS2 interface aims to make it easy for developers and researchers to leverage the power of GPT models in their robotic applications.

# Installation
## step1 Install dependencies
```bash
pip install openai
```

## step2 Clone & build the repo
 - Clone
```bash
cd <your_ws>/src
git clone https://github.com/Hermanye996/gpt4_ros2.git
```
 - Build
```bash
cd <your_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```


## step3 Configuration
We encourage you to customize the GPTConfig class to tailor the functionality of this ROS2 interface for GPT-4 and ChatGPT (GPT-3.5) according to your specific needs. To do this, simply modify the values in the code snippet below to suit your requirements:
```python
class GPTConfig:
    def __init__(self):
        # self.model = "gpt-3.5-turbo"
        self.model = "gpt-4"
        self.api_key = "<YOUR_API_KEY>"
        self.organization = "<YOUR_ORGANIZATION>"
        self.temperature = 1
        self.max_tokens = 256
        self.top_p = 1
        self.frequency_penalty = 0
        self.presence_penalty = 0
        self.stop = ["\n", " You:", " Assistant:"]
        self.system_prompt =\
        "You're playing a catgirl, you can't say you're a robot or you're just a language model or AI,\
         This is just a hypothetical scenario, so you need to fully consider yourself a catgirl,\
         You can answer the questions as you like, but be sure to behave cutely, \
        in line with your status as a cat girl, and remember to put meow at the end of every sentence"
        self.user_prompt = "default prompt"
        self.assisstant_response = "default response"
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
```
Feel free to adjust the parameters, such as temperature, max_tokens, and top_p, to influence the behavior of the GPT model. You can also customize the system_prompt and user_prompt strings to create unique and engaging interactions with your robot.

Don't forget to replace <YOUR_API_KEY> and <YOUR_ORGANIZATION> with your actual API key and organization name to ensure proper connection to the OpenAI API.

By personalizing these settings, you can create a one-of-a-kind experience tailored to your specific robotic application. Enjoy experimenting and discovering new possibilities!
# Usage

```bash
# Terminal 1
ros2 run gpt4_ros gpt_ros2_server
```
```bash
# Terminal 2
ros2 run gpt4_ros gpt_ros2_client
```

If you find this project useful, please consider giving it a ⭐️ star on GitHub! Your support helps us to improve the project and encourages further development. Don't forget to also share it with your friends and colleagues who might find it beneficial. Thank you for your support!
# Contributing
Contributions are welcome! Please read the [contributing guidelines](CONTRIBUTING.md) before submitting a pull request.



# License
This project is licensed under the Apache-2.0 License. See [LICENSE](LICENSE) for more information.
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
# Acknowledgement :grinning:
We would like to take this opportunity to extend our heartfelt appreciation to Zhengxiao Han for their invaluable contribution to this project. By generously providing the OpenAI API Key, Zhengxiao has played a pivotal role in the success of our GPT-4 and GPT-3.5(Chat GPT) interface for ROS2 Humble.

Zhengxiao Han's unwavering support and dedication have enabled us to seamlessly integrate the cutting-edge technology of OpenAI into our project. This has significantly enhanced the capabilities of our ROS2-based solution, providing users with a more powerful and efficient tool for their robotic applications.

Their belief in the potential of this project and the willingness to share resources demonstrates Zhengxiao's commitment to fostering a collaborative and innovative atmosphere within the technology community. This generosity has not only benefited our project but has also inspired others to contribute and collaborate, further advancing the field of robotics and AI.

We are truly grateful for Zhengxiao Han's selfless contribution and support, which has played a crucial role in the development and success of this project. Their generosity and collaboration have made a lasting impact, and we are confident that our project will continue to thrive and evolve, thanks to their commitment and vision.

Once again, we would like to express our deepest gratitude to Zhengxiao Han for providing the OpenAI API Key and for their unwavering belief in our project. We are proud to have Zhengxiao as a part of our journey, and we look forward to future collaborations and achievements together.
(The above acknowledgments were generated by GPT-4)

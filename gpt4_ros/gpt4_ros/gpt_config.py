class GPTConfig:
    def __init__(self):
        # self.model = "gpt-3.5-turbo"
        self.mode = "gpt-4"
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

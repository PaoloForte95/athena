#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import os
import base64
from openai import OpenAI
from functools import lru_cache
from cv_bridge import CvBridge
import base64
import cv2
from sensor_msgs.msg import Image
from athena_msgs.srv import GenerateProblemFile
from google import genai
import PIL.Image
from pathlib import Path


GPT_MODEL="gpt-4.1"
GEMINI_MODEL = "gemini-3-flash-preview"
open_ai_key = os.environ["OPENAI_API_KEY"]
gemini_api_key=os.environ["GEMINI_API_KEY"]
class VlmApi:
    def __init__(self, system_prompt, instruction, output_file = "problem.pddl"):

        self.output_file = output_file
        self.prompt = None
        self.instruction = None
        self.logger = rclpy.logging.get_logger('VlmApi')
        with open(system_prompt, "r") as file:
            # Read the entire content of the file
            self.prompt = file.read()

        with open(instruction, "r") as command_file:
            # Read the entire content of the file
            self.instruction = command_file.read()

        self.openai_client = OpenAI(api_key=open_ai_key)  # CHANGED
        self.gemini_client = genai.Client(api_key=gemini_api_key)

    # Function to encode the image
    def encode_image(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    # Path to your image
    @lru_cache()
    def analyze_image(self,img_path, user_prompt, prompt):
        base64_image = self.encode_image(img_path)

        response = self.openai_client.responses.create(
        model=GPT_MODEL,
        temperature=0.0,
        input=[
            {"role": "system", "content": [{"type": "input_text", "text": prompt}]},
            {"role": "user", "content": [
                {"type": "input_text", "text": user_prompt},
                {"type": "input_image", "image_url": f"data:image/png;base64,{base64_image}"}
            ]}
        ],
        )
        return response.output_text
        
    def generateProblemFile(self, image, model = "ChatGpt"):
        if "Gemini" in model:
            self.logger.info(f"Generating planning problem using Gemini")
            response = self.gemini_client.models.generate_content(
                model=GEMINI_MODEL,
                contents=[self.prompt, image])
            response = response.text
        if "ChatGpt" in model:
            self.logger.info(f"Generating planning problem using ChatGPT")
            response = self.analyze_image(image, self.instruction, self.prompt)
        self.logger.info(response)
        parse = self.export_file(response, self.output_file)
        self.logger.info(parse)
        return self.output_file
    

    def export_file(self,input_text, output_file):
        """
        Extracts the PDDL section from a string where the PDDL code is embedded within other text
        and saves it to a specified file path, replacing any existing content.

        Parameters:
        input_text (str): The string from which to extract the PDDL content.
        output_file (str): The path to the file where the PDDL content should be saved.
        
        Returns:
        str: Confirmation message about saving, or an error message if not found.
        """

        start_index = input_text.find("(define")
        if start_index == -1:
            return "PDDL content not found."

        end_index = input_text.rfind(")") + 1
        if end_index == 0:
            return "PDDL content not found."

        pddl_content = input_text[start_index:end_index]

        with open(output_file, 'w') as file:
            file.write(pddl_content)

        return f"PDDL content successfully saved to {output_file}"
   

class VlmApiNode(Node):

    def __init__(self):
        super().__init__("VlmApi")
        
        self.srv = self.create_service(GenerateProblemFile, 'generate_problem_file', self.compute_problem_file_callback)
      
        
        
        
    def compute_problem_file_callback(self, request, response):
        
        image = request.image_file.data
        cwd = Path.cwd()
        image_path = str(cwd) + image
        outfile = request.output_name.data
        self.VlmApi = VlmApi(request.prompt.data, request.instruction.data, outfile)
        filename = self.VlmApi.generateProblemFile("/home/pofe/planning_ws/captured_image.png")
        msg = String()
        msg.data = filename
        response.problem_file = msg
        self.get_logger().info('Problem %s file created!' %filename)
        return response



def main(args=None):
    rclpy.init(args=args)

    VlmApi = VlmApiNode()
    try:
        rclpy.spin(VlmApi)
    except KeyboardInterrupt:
        pass
    
    VlmApi.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

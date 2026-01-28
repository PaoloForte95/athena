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

GPT_MODEL="gpt-4o"
GEMINI_MODEL = "gemini-2.5-flash"
open_ai_key = os.environ["OPEN_API_KEY"]
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

        #self.openai_client = OpenAI(api_key=open_ai_key)
        self.gemini_client = genai.Client(api_key=gemini_api_key)
    # Function to encode the image
    def encode_image(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    # Path to your image
    @lru_cache()
    def analyze_image(self,img_path, user_prompt, prompt):
        base64_image = self.encode_image(img_path)
        response = self.openai_client.chat.completions.create(
        model=GPT_MODEL,
        messages=[
            {"role": "system", "content": prompt},
            {"role": "user", "content": [
            {"type": "text", "text": user_prompt},
            {"type": "image_url", "image_url": {
                "url": f"data:image/png;base64,{base64_image}"}
            }
            ]}
        ],
        temperature=0.0,
         )
        
        response = response.choices[0].message.content
        return response
    
    def generateProblemFile(self, image_path, model = "Gemini"):
        image = PIL.Image.open(image_path)
        if "Gemini" in model:
            self.logger.info(f"Generating planning problem using Gemini")
            response = self.gemini_client.models.generate_content(
                model=GEMINI_MODEL,
                contents=[self.prompt, image])
            response = response.text
        elif "ChatGpt" in model:
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

        # ### Creating an image with GPT output
        # lines = pddl_content.split('\n')
        # text_image = np.ones((1200, 800), dtype=np.uint8)*255
        # text_size = 0.65
        # text_width = 1
        # text_color = (0,0,255)
        # text_font = cv2.FONT_HERSHEY_SIMPLEX
        # text_line = cv2.LINE_AA
        # text_height = 50

        # for line in lines:
        #     text_image = cv2.putText(text_image, line, (5, text_height), text_font, text_size, text_color, text_width, text_line)
        #     text_height += 25
        # PILImage.fromarray(text_image).save("plan_image.png")

        with open(output_file, 'w') as file:
            file.write(pddl_content)

        return f"PDDL content successfully saved to {output_file}"
   

class VlmApiNode(Node):

    def __init__(self):
        super().__init__("VlmApi")
        
        self.image_count = 1 #307  
        self.srv = self.create_service(GenerateProblemFile, 'generate_problem_file', self.compute_problem_file_callback)
        self.image = None
        self.image_sub =  self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_listener_callback,
            10)

      
        
        
        
    def compute_problem_file_callback(self, request, response):
        
        try:
            bridge = CvBridge()
            self.get_logger().info(f'Creating problem')

            if request.image_file.data.strip() != "":
                image_path = request.image_file.data
            
            else:
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')

                # Create a file name
                image_name = f'problem1.png'
                
                image_path = os.path.join("./", image_name)
                # Save the image
                cv2.imwrite(image_path, cv_image)
                
            outfile = f'problem1.pddl'
    
            self.VlmApi = VlmApi(request.prompt.data, request.instruction.data, outfile)
            filename = self.VlmApi.generateProblemFile(image_path)
            msg = String()
            msg.data = filename
            response.problem_file = msg
            self.get_logger().info('Problem %s file created!' %filename)
        
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
        return response



    def image_listener_callback(self, msg):
        self.image = msg


def main(args=None):
    rclpy.init(args=args)

    VlmApi = VlmApiNode()
    try:
        rclpy.spin(VlmApi)
    except KeyboardInterrupt:
        pass
    
    VlmApi.destroy_node()
    rclpy.try_shutdown()
    #vlm = VlmApi("prompt.txt", "instruction.txt")
    #vlm.generateProblemFile("problem1.png")


if __name__ == '__main__':
    main()

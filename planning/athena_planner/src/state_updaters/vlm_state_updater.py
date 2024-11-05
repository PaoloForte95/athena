#!/usr/bin/env python3

import google.generativeai as genai
import os
import argparse

genai.configure(api_key=os.environ["API_KEY"])

instruction_file = "instruction.txt"
domain_file = "src/athena/planning/athena_example/PDDL/Object_Arrangement/domain.pddl"
problem_file = "src/athena/planning/athena_example/PDDL/Object_Arrangement/problem1.pddl"


with open(instruction_file, "r") as file:
    # Read the entire content of the file
    instruction = file.read()

model = genai.GenerativeModel("gemini-1.5-flash", system_instruction=instruction)

chat = model.start_chat(
    history=[
        {"role": "user", "parts": "Hello"},
        {"role": "model", "parts": "Great to meet you. What would you like to know?"},
    ]
)


def updateState(action):
    with open(domain_file, "r") as file:
    # Read the entire content of the file
        domain = file.read()

    with open(problem_file, "r") as file:
        # Read the entire content of the file
        problem = file.read()

    chat.send_message(["This is the planning Domain: " + domain])
    chat.send_message(["This is the planning problem: " + problem])
    response = chat.send_message(action)
    print(response.text)
    return response.text



def main(args):
    if args.verbose:
        print(f"Applying action: {args.action}")
        #print(f"Planning domain file: {args.domain}")
        #print(f"Planning problem file: {args.problem}")


    
    #domain_file = {args.domain}
    #problem_file = {args.problem}
    with open(domain_file, "r") as file:
        # Read the entire content of the file
        domain = file.read()

    with open(problem_file, "r") as file:
        # Read the entire content of the file
        problem = file.read()

    chat.send_message(["This is the planning Domain: " + domain])
    chat.send_message(["This is the planning problem: " + problem])
    #response = chat.send_message("Give me the initial state")
    #print(response.text)

    updateState({args.action})

    # prompt_action = " Apply action: move robot1 home1 wp_1"
    # updateState(prompt_action)


    # prompt_action = " Apply action: (PICK ROBOT1 RED_PLATE WP1S)"
    # updateState(prompt_action)

    # prompt_action = " Apply action: (MOVE ROBOT1 WP1S WP1F) "
    # updateState(prompt_action)

    # prompt_action = " Apply action: (PLACE ROBOT1 RED_PLATE WP1F)"
    # updateState(prompt_action)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="A script that processes files")
    # Add arguments
    #parser.add_argument('-d', '--domain', type=str, required=True, help="Path to the planning domain file")
    #parser.add_argument('-p', '--problem', type=str, required=True, help="Path to the planning problem file")
    parser.add_argument('-a', '--action', type=str, required=True, help="Action to apply")
    parser.add_argument('-v', '--verbose', action='store_true', help="Enable verbose mode")
    
    args = parser.parse_args()
    main(args)

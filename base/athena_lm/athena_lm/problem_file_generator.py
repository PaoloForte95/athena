#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import os
import re
import time
from pathlib import Path
from openai import OpenAI
from functools import lru_cache

from athena_msgs.srv import GenerateProblemFile
from google import genai
import ollama

GPT_MODEL = "gpt-5.2"
GEMINI_MODEL = "gemini-3-flash-preview"
OLLAMA_MODEL = "qwen3.5"

open_ai_key = os.environ["OPENAI_API_KEY"]
gemini_api_key = os.environ["GEMINI_API_KEY"]

LOGICAL_KEYWORDS = {"and", "not", "or", "forall", "exists", "imply", "="}

SYSTEM_PROMPT = (
    "You write PDDL planning goals. "
    "You are given a planning domain, an instruction in plain language, "
    "and the list of available objects. "
    "Produce only the goal section of a PDDL problem. "
    "Use only the predicates defined in the domain and only the objects provided. Use only the types from the domain file "
    "Return a single block that starts with (:goal and nothing else."
)

INIT_CHECK_PROMPT = (
    "You review the initial state of a PDDL problem. "
    "You are given a planning domain, the objects, and the current initial state. "
    "List only initial state facts that are clearly missing for this scene. "
    "Use only predicates defined in the domain and only the listed objects. "
    "Return each fact on its own line as a PDDL fact like (predicate arg1 arg2). "
    "If nothing is missing, return NONE."
)


class VlmApi:
    def __init__(self, domain_file, output_file="problem.pddl", check_init=False, max_retries=3):

        self.output_file = output_file
        self.prompt = SYSTEM_PROMPT
        self.check_init = check_init
        self.max_retries = max_retries
        self.logger = rclpy.logging.get_logger('VlmApi')

        with open(domain_file, "r") as file:
            self.domain = file.read()

        self.predicates = self.domain_predicates(self.domain)

        self.openai_client = OpenAI(api_key=open_ai_key)
        self.gemini_client = genai.Client(api_key=gemini_api_key)

    def balanced_from(self, text, start):
        depth = 0
        for i in range(start, len(text)):
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
                if depth == 0:
                    return text[start:i + 1]
        return ""

    def domain_predicates(self, domain_text):
        text = re.sub(r";.*", "", domain_text)
        start = text.find("(:predicates")
        if start == -1:
            return {}
        block = self.balanced_from(text, start)
        declared = {}
        for atom in re.finditer(r"\((\S+)([^()]*)\)", block):
            name = atom.group(1).lower()
            if name == ":predicates":
                continue
            arity = sum(1 for t in atom.group(2).split() if t.startswith("?"))
            declared[name] = arity
        return declared

    def object_names(self, objects):
        names = set()
        for line in objects:
            left = line.replace("(", " ").replace(")", " ").split(" - ")[0]
            for token in left.split():
                if token != "-":
                    names.add(token.lower())
        return names

    def validate_goal(self, goal, objects):
        errors = []
        names = self.object_names(objects)
        atoms = list(re.finditer(r"\((\S+)([^()]*)\)", goal))
        found = False
        for atom in atoms:
            pred = atom.group(1).lower()
            args = atom.group(2).split()
            if pred in LOGICAL_KEYWORDS or pred.startswith(":"):
                continue
            found = True
            if pred not in self.predicates:
                errors.append(f"predicate '{pred}' is not declared in the domain")
                continue
            if len(args) != self.predicates[pred]:
                errors.append(
                    f"predicate '{pred}' expects {self.predicates[pred]} arguments, got {len(args)}"
                )
            for arg in args:
                if arg.lower() not in names:
                    errors.append(f"argument '{arg}' of '{pred}' is not one of the available objects")
        if not found:
            errors.append("the goal contains no predicate")
        return sorted(set(errors))

    def build_user_prompt(self, instruction, objects):
        objects_text = "\n".join(objects)
        return (
            f"Planning domain:\n{self.domain}\n\n"
            f"Instruction:\n{instruction}\n\n"
            f"Available objects:\n{objects_text}\n\n"
            "Write only the PDDL goal section for the instruction above. "
            "Use only the listed objects and only predicates defined in the domain. "
            "Return a single block starting with (:goal and nothing else."
        )

    def build_init_check_prompt(self, instruction, objects, init):
        objects_text = "\n".join(objects)
        init_text = "\n".join(init)
        return (
            f"Planning domain:\n{self.domain}\n\n"
            f"Instruction:\n{instruction}\n\n"
            f"Objects:\n{objects_text}\n\n"
            f"Current initial state:\n{init_text}\n\n"
            "List any initial state facts that are clearly missing for this scene. "
            "Use only predicates from the domain and only the listed objects. "
            "Return each fact on its own line as a PDDL fact. "
            "If nothing is missing, return NONE."
        )

    @lru_cache()
    def analyze_text(self, user_prompt, prompt):
        response = self.openai_client.responses.create(
            model=GPT_MODEL,
            temperature=0.0,
            input=[
                {"role": "system", "content": [{"type": "input_text", "text": prompt}]},
                {"role": "user", "content": [{"type": "input_text", "text": user_prompt}]},
            ],
        )
        return response.output_text

    @lru_cache()
    def analyze_text_ollama(self, user_prompt, prompt):
        response = ollama.generate(
            model=OLLAMA_MODEL,
            prompt=f"{prompt}\n\n{user_prompt}",
            options={"temperature": 0.0},
        )
        return response["response"]

    def call_model(self, user_prompt, system_prompt, model):
        if "Gemini" in model:
            response = self.gemini_client.models.generate_content(
                model=GEMINI_MODEL,
                contents=[system_prompt, user_prompt])
            return response.text
        elif "ChatGpt" in model:
            return self.analyze_text(user_prompt, system_prompt)
        elif "Ollama" in model:
            return self.analyze_text_ollama(user_prompt, system_prompt)
        return ""

    def generate_goal(self, instruction, objects, model="ChatGpt"):
        self.logger.info("Generating goal using %s" % model)
        base_prompt = self.build_user_prompt(instruction, objects)
        self.logger.info("User prompt:\n%s" % base_prompt)

        prompt = base_prompt
        llm_time = 0.0
        attempts = 0
        goal = ""
        errors = []

        while attempts < self.max_retries:
            attempts += 1
            self.logger.info("Goal generation attempt %d/%d" % (attempts, self.max_retries))
            t_llm = time.perf_counter()
            response = self.call_model(prompt, self.prompt, model)
            llm_time += time.perf_counter() - t_llm
            self.logger.info(response)

            goal = self.extract_goal(response)
            errors = self.validate_goal(goal, objects)
            if not errors:
                break

            for err in errors:
                self.logger.warn("Validation: %s" % err)
            error_list = "\n".join("- %s" % e for e in errors)
            prompt = (
                "%s\n\nYour previous answer was:\n%s\n\n"
                "It has these errors:\n%s\n\n"
                "Rewrite the goal and fix every error. Use only the declared predicates "
                "and the listed objects. Return a single block starting with (:goal and nothing else."
                % (base_prompt, response, error_list)
            )

        self.logger.info("LLM call time: %.2fs over %d attempt(s)" % (llm_time, attempts))
        self.log_timing(llm_time, attempts)

        if errors:
            self.logger.error("Goal rejected after %d attempts: %s" % (attempts, "; ".join(errors)))
            return None
        return goal

    def check_missing_init(self, instruction, objects, init, model="ChatGpt"):
        self.logger.info("Checking initial state using %s" % model)
        user_prompt = self.build_init_check_prompt(instruction, objects, init)
        self.logger.info("User prompt:\n%s" % user_prompt)
        response = self.call_model(user_prompt, INIT_CHECK_PROMPT, model)
        self.logger.info(response)
        return self.extract_facts(response)

    def extract_goal(self, input_text):
        start_index = input_text.find("(:goal")
        if start_index == -1:
            return "(:goal (and ))"
        block = self.balanced_from(input_text, start_index)
        return block if block else "(:goal (and ))"

    def extract_facts(self, input_text):
        facts = []
        depth = 0
        start = -1
        for i, char in enumerate(input_text):
            if char == "(":
                if depth == 0:
                    start = i
                depth += 1
            elif char == ")":
                if depth > 0:
                    depth -= 1
                    if depth == 0 and start != -1:
                        facts.append(input_text[start:i + 1])
                        start = -1
        return facts

    def is_valid_fact(self, fact):
        inner = fact.strip()
        if not (inner.startswith("(") and inner.endswith(")")):
            return False
        return len(inner[1:-1].split()) >= 2

    def merge_init(self, init, extra_facts):
        merged = [f for f in init if self.is_valid_fact(f)]
        for fact in extra_facts:
            if self.is_valid_fact(fact) and fact not in merged:
                merged.append(fact)
                self.logger.info("Added missing init fact: %s" % fact)
        return merged

    def build_problem_file(self, objects, init, goal):
        objects_text = "\n    ".join(objects)
        init_text = "\n    ".join(init)
        return (
            f"(define (problem pb01)\n"
            f"  (:domain domain_pddl)\n"
            f"  (:objects\n    {objects_text}\n  )\n"
            f"  (:init\n    {init_text}\n  )\n"
            f"  {goal}\n"
            f")\n"
        )

    def log_timing(self, llm_time, attempts):
        csv_path = Path(self.output_file).parent / "generation_times.csv"
        new_file = not csv_path.exists()
        with open(csv_path, "a", encoding="utf-8") as f:
            if new_file:
                f.write("file,llm_time_s,attempts\n")
            f.write(f"{Path(self.output_file).name},{llm_time:.3f},{attempts}\n")

    def generateProblemFile(self, instruction, objects, init, model="ChatGpt"):
        goal = self.generate_goal(instruction, objects, model)
        if goal is None:
            return "", ""

        if self.check_init:
            extra_facts = self.check_missing_init(instruction, objects, init, model)
            init = self.merge_init(init, extra_facts)

        problem = self.build_problem_file(objects, init, goal)

        with open(self.output_file, 'w') as file:
            file.write(problem)

        self.logger.info("PDDL problem saved to %s" % self.output_file)
        return self.output_file, problem


class VlmApiNode(Node):

    def __init__(self):
        super().__init__("VlmApi")

        self.declare_parameter("output_file", "problem.pddl")
        self.declare_parameter("model", "ChatGpt")
        self.declare_parameter("check_init", False)
        self.declare_parameter("max_retries", 3)

        self.problem_pub = self.create_publisher(String, 'generated_problem', 10)
        self.srv = self.create_service(GenerateProblemFile, 'generate_problem_file', self.compute_problem_file_callback)

    def compute_problem_file_callback(self, request, response):
        instruction = request.instruction
        objects = list(request.objects)
        init = list(request.init)
        domain_file = request.domain

        self.get_logger().info('Instruction: %s' % instruction)
        self.get_logger().info('Domain file: %s' % domain_file)
        self.get_logger().info('Objects received: %d' % len(objects))
        for obj in objects:
            self.get_logger().info('Object: %s' % obj)
        self.get_logger().info('Init facts received: %d' % len(init))
        for fact in init:
            self.get_logger().info('Init fact: %s' % fact)

        output_file = self.get_parameter("output_file").get_parameter_value().string_value
        model = self.get_parameter("model").get_parameter_value().string_value
        check_init = self.get_parameter("check_init").get_parameter_value().bool_value
        max_retries = self.get_parameter("max_retries").get_parameter_value().integer_value

        vlm = VlmApi(domain_file, output_file, check_init, max_retries)
        filename, problem = vlm.generateProblemFile(instruction, objects, init, model)

        msg = String()
        msg.data = filename
        response.problem_file = msg
        if filename:
            problem_msg = String()
            problem_msg.data = problem
            self.problem_pub.publish(problem_msg)
            self.get_logger().info('Problem %s file created and published on generated_problem' % filename)
        else:
            self.get_logger().error('No problem file created: goal rejected')
        return response


def main(args=None):
    rclpy.init(args=args)

    node = VlmApiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
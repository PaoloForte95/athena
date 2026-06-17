#!/usr/bin/env python3

import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openai import OpenAI
import ollama

from athena_msgs.srv import GenerateDomain


class PddlDomainServer(Node):
    def __init__(self):
        super().__init__("domain_file_generator_node")

        self.declare_parameter("backend", "openai")
        self.declare_parameter("openai_model", "gpt-5.2")
        self.declare_parameter("ollama_model", "qwen3.5")
        self.declare_parameter("output_file", "domain.pddl")

        self.pub = self.create_publisher(String, "/generated_domain", 10)
        self.create_service(GenerateDomain, "generate_domain", self.handle_generate)

        self.get_logger().info("Domain generator ready")

    def call_llm(self, backend, openai_model, ollama_model, prompt):
        if backend == "openai":
            client = OpenAI()
            resp = client.responses.create(
                model=openai_model,
                input=prompt,
            )
            return resp.output_text.strip()
        else:
            available = [m["model"] for m in ollama.list()["models"]]
            if ollama_model not in available:
                self.get_logger().info(f"Pulling model '{ollama_model}'...")
                ollama.pull(ollama_model)
                self.get_logger().info(f"Model '{ollama_model}' ready.")
            response = ollama.generate(
                model=ollama_model,
                prompt=prompt,
            )
            return response["response"].strip()

    def handle_generate(self, request, response):
        backend = str(self.get_parameter("backend").value).lower()
        openai_model = str(self.get_parameter("openai_model").value)
        ollama_model = str(self.get_parameter("ollama_model").value)
        output_file = str(self.get_parameter("output_file").value)

        types_block = request.types
        predicate_library_text = request.predicates
        action_library_text = request.actions

        if backend not in ("openai", "ollama"):
            response.success = False
            response.message = f"Unknown backend '{backend}'. Use 'openai' or 'ollama'."
            self.get_logger().error(response.message)
            return response

        if backend == "openai" and not os.environ.get("OPENAI_API_KEY"):
            response.success = False
            response.message = "OPENAI_API_KEY not set"
            self.get_logger().error(response.message)
            return response

        if not action_library_text:
            response.success = False
            response.message = "No actions provided in request"
            self.get_logger().error(response.message)
            return response

        # =========================
        # STAGE 1: generate predicates
        # =========================
        predicates_prompt = f"""
You are a robotics + PDDL engineer.

Task: produce the concrete PDDL predicate declarations for the (:predicates ...) section of a PDDL domain.

Constraints:
- Use ONLY the types listed below. Do NOT invent new types.
- Use ONLY the predicate names from the predicate library. Do NOT invent new predicates.
- For each predicate in the library, write its concrete typed declaration matching its abstract arguments. Replace role names (like agent, object, locatable) with the most appropriate concrete type from the type list.
- Output ONLY the predicate declarations, one per line, in the form:
    (predicate_name ?arg0 - type0 ?arg1 - type1 ...)
- No extra text, no comments, no parentheses around the whole block.

--- types ---
{types_block}
--- end ---

--- predicate library ---
{predicate_library_text}
--- end ---
""".strip()

        self.get_logger().info("Stage 1: generating predicates...")
        try:
            predicates_block = self.call_llm(backend, openai_model, ollama_model, predicates_prompt)
        except Exception as e:
            response.success = False
            response.message = f"Stage 1 LLM call failed ({backend}): {e}"
            self.get_logger().error(response.message)
            return response

        print("=" * 60)
        print("STAGE 1 OUTPUT (predicates):")
        print("=" * 60)
        print(predicates_block)
        print("=" * 60)

        # =========================
        # STAGE 2: generate actions and full domain
        # =========================
        actions_prompt = f"""
You are a robotics + PDDL engineer.

Generate a full PDDL domain file following EXACTLY the skeleton below.

Constraints:
- Use ONLY the types and predicates provided. Do NOT invent new ones.
- Define one (:action ...) block for EACH action listed in the actions section.
- Fill each action's :parameters, :precondition, and :effect using ONLY the provided types and predicates.
- For negation, ALWAYS use (not (predicate ...)). Never use ~ or !.
- Allowed PDDL features (matching :requirements):
    * :typing — use typed parameters like (?r - robot)
    * :negative-preconditions — use (not (predicate ...)) for negation
  Do NOT use any other features (no :conditional-effects, no :disjunctive-preconditions, no numeric fluents, no :equality).

Return ONLY the full domain file text, no explanations.

--- skeleton to follow ---
(define (domain domain_pddl)
    (:requirements :negative-preconditions :typing)
    (:types
    )
    (:predicates
    )

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; ACTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:action action_name
        :parameters ()
        :precondition
            (and
            )
        :effect
            (and
            )
    )
)
--- end ---

--- types ---
{types_block}
--- end ---

--- predicates (use these EXACTLY as given) ---
{predicates_block}
--- end ---

--- actions to build ---
{action_library_text}
--- end ---
""".strip()

        self.get_logger().info("Stage 2: generating actions and full domain...")
        try:
            out_text = self.call_llm(backend, openai_model, ollama_model, actions_prompt)
        except Exception as e:
            response.success = False
            response.message = f"Stage 2 LLM call failed ({backend}): {e}"
            self.get_logger().error(response.message)
            return response

        print("=" * 60)
        print("STAGE 2 OUTPUT (full domain):")
        print("=" * 60)
        print(out_text)
        print("=" * 60)

        try:
            out_path = Path(output_file)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(out_text, encoding="utf-8")
            self.get_logger().info(f"Saved domain: {out_path}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to write file: {e}"
            self.get_logger().error(response.message)
            return response

        msg = String()
        msg.data = out_text
        self.pub.publish(msg)
        self.get_logger().info(f"Published /generated_domain (backend={backend})")

        response.success = True
        response.path = str(out_path)
        response.message = "ok"
        return response


def main():
    rclpy.init()
    node = PddlDomainServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
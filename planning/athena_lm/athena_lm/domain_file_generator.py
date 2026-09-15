#!/usr/bin/env python3

import os
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import yaml
from openai import OpenAI
import ollama

from athena_msgs.srv import GenerateDomain

LOGICAL_KEYWORDS = {"and", "not", "or", "forall", "exists", "when", "imply", "="}


class PddlDomainServer(Node):
    def __init__(self):
        super().__init__("domain_file_generator_node")

        self.declare_parameter("backend", "openai")
        self.declare_parameter("openai_model", "gpt-5.2")
        self.declare_parameter("ollama_model", "qwen3.5")
        self.declare_parameter("output_file", "domain.pddl")
        self.declare_parameter("capabilities", "")
        self.declare_parameter("max_retries", 3)

        self.pub = self.create_publisher(String, "/generated_domain", 10)
        self.create_service(GenerateDomain, "generate_domain", self.handle_generate)

        self.get_logger().info("Domain generator ready")

    def read_text(self, p: str) -> str:
        return Path(p).read_text(encoding="utf-8")

    def extract_capabilities(self, robot_cfg: dict) -> set:
        caps = robot_cfg.get("capabilities", {})
        if isinstance(caps, dict):
            return {k for k, v in caps.items() if v}
        if isinstance(caps, list):
            return set(caps)
        return set()

    def action_is_available(self, action, robot_capabilities: set) -> bool:
        if not set(action.requirements).issubset(robot_capabilities):
            return False
        if action.any_of and not (set(action.any_of) & robot_capabilities):
            return False
        return True

    def build_predicate_block(self, predicates) -> str:
        decls = []
        for p in predicates:
            name, _, args = p.partition(" ")
            args = args.strip()
            if args.startswith("(") and args.endswith(")"):
                args = args[1:-1]
            decls.append(f"({name} {args.strip()})")
        return "\n        ".join(decls)

    def declared_types(self, types) -> set:
        declared = set()
        for line in types:
            for token in line.replace("(", " ").replace(")", " ").split():
                if token != "-":
                    declared.add(token.lower())
        declared.add("object")
        return declared

    def declared_predicates(self, predicates) -> dict:
        declared = {}
        for p in predicates:
            name, _, args = p.partition(" ")
            args = args.strip()
            if args.startswith("(") and args.endswith(")"):
                args = args[1:-1]
            arity = sum(1 for t in args.split() if t.startswith("?"))
            declared[name.strip().lower()] = arity
        return declared

    def balanced_from(self, text: str, start: int) -> str:
        depth = 0
        for i in range(start, len(text)):
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
                if depth == 0:
                    return text[start:i + 1]
        return ""

    def split_action_blocks(self, text: str) -> list:
        blocks = []
        pos = 0
        while True:
            start = text.find("(:action", pos)
            if start == -1:
                break
            block = self.balanced_from(text, start)
            if not block:
                break
            blocks.append(block)
            pos = start + len(block)
        return blocks

    def extract_action_blocks(self, text: str) -> str:
        blocks = self.split_action_blocks(text)
        return "\n\n    ".join(blocks)

    def section(self, block: str, key: str) -> str:
        start = block.find(key)
        if start == -1:
            return ""
        paren = block.find("(", start)
        if paren == -1:
            return ""
        return self.balanced_from(block, paren)

    def validate_actions(self, actions_text, requested_names, types_declared, preds_declared) -> list:
        errors = []
        blocks = self.split_action_blocks(actions_text)
        if not blocks:
            return ["no (:action ...) block found"]

        seen = set()
        for block in blocks:
            match = re.search(r"\(:action\s+(\S+)", block)
            if not match:
                errors.append("action block without a name")
                continue
            name = match.group(1).lower()
            seen.add(name)
            if name not in requested_names:
                errors.append(f"action '{name}' was not requested")

            params_text = self.section(block, ":parameters")
            variables = set()
            tokens = params_text.strip("()").split()
            i = 0
            pending = []
            while i < len(tokens):
                tok = tokens[i]
                if tok.startswith("?"):
                    pending.append(tok.lower())
                    variables.add(tok.lower())
                elif tok == "-":
                    i += 1
                    if i < len(tokens):
                        typ = tokens[i].lower()
                        if typ not in types_declared:
                            errors.append(f"action '{name}': type '{typ}' is not declared")
                    pending = []
                i += 1

            for key in (":precondition", ":effect"):
                body = self.section(block, key)
                if not body:
                    errors.append(f"action '{name}': missing {key}")
                    continue
                for atom in re.finditer(r"\((\S+)([^()]*)\)", body):
                    pred = atom.group(1).lower()
                    args = atom.group(2).split()
                    if pred in LOGICAL_KEYWORDS or pred.startswith(":"):
                        continue
                    if pred not in preds_declared:
                        errors.append(f"action '{name}': predicate '{pred}' is not declared")
                        continue
                    if len(args) != preds_declared[pred]:
                        errors.append(
                            f"action '{name}': predicate '{pred}' expects "
                            f"{preds_declared[pred]} arguments, got {len(args)}"
                        )
                    for arg in args:
                        if arg.lower() not in variables:
                            errors.append(
                                f"action '{name}': argument '{arg}' of '{pred}' is not a parameter"
                            )

        for name in requested_names:
            if name not in seen:
                errors.append(f"action '{name}' is missing")

        return sorted(set(errors))

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

    def log_timing(self, output_file: str, llm_time: float, attempts: int):
        csv_path = Path(output_file).parent / "generation_times.csv"
        new_file = not csv_path.exists()
        with open(csv_path, "a", encoding="utf-8") as f:
            if new_file:
                f.write("file,llm_time_s,attempts\n")
            f.write(f"{Path(output_file).name},{llm_time:.3f},{attempts}\n")

    def handle_generate(self, request, response):
        backend = str(self.get_parameter("backend").value).lower()
        openai_model = str(self.get_parameter("openai_model").value)
        ollama_model = str(self.get_parameter("ollama_model").value)
        output_file = str(self.get_parameter("output_file").value)
        capabilities = str(self.get_parameter("capabilities").value)
        max_retries = int(self.get_parameter("max_retries").value)

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

        robot_cfg = yaml.safe_load(self.read_text(capabilities))
        capabilities_set = self.extract_capabilities(robot_cfg)
        self.get_logger().info(f"Extracted capabilities: {capabilities_set}")

        available_actions = [
            a for a in request.actions if self.action_is_available(a, capabilities_set)
        ]
        if not available_actions:
            response.success = False
            response.message = "No actions match the robot capabilities"
            self.get_logger().error(response.message)
            return response

        types_block = "\n".join(request.types)
        types_indented = "\n        ".join(request.types)
        predicates_block = self.build_predicate_block(request.predicates)
        action_library_text = "\n".join(f"- {a.name}" for a in available_actions)

        types_declared = self.declared_types(request.types)
        preds_declared = self.declared_predicates(request.predicates)
        requested_names = {a.name.lower() for a in available_actions}

        print("=" * 60)
        print("TYPES:")
        print("=" * 60)
        print(types_block)
        print("=" * 60)
        print("PREDICATES:")
        print("=" * 60)
        print(predicates_block)
        print("=" * 60)
        print("ACTIONS TO BUILD:")
        print("=" * 60)
        print(action_library_text)
        print("=" * 60)

        base_prompt = f"""
You are a robotics + PDDL engineer.

Write ONLY the (:action ...) blocks for the actions listed below.

Constraints:
- Use ONLY the types and predicates provided. Do NOT invent new ones.
- Write one (:action ...) block for EACH action listed.
- Fill each action's :parameters, :precondition, and :effect using ONLY the provided types and predicates.
- Every predicate you use must appear with the exact number and types of arguments given in the predicate list.
- Do NOT use the same variable twice in one predicate unless that is truly intended.
- If an action is meant to remove a condition, put (not (predicate ...)) in its :effect.
- For negation, ALWAYS use (not (predicate ...)). Never use ~ or !.
- Only :typing and :negative-preconditions are allowed. No conditional effects,
  no disjunctive preconditions, no numeric fluents, no equality.
- Return ONLY the action blocks. Do NOT write the domain header, the (:types ...)
  section, the (:predicates ...) section, or any explanation.

--- format of each block ---
    (:action action_name
        :parameters ()
        :precondition
            (and
            )
        :effect
            (and
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

        prompt = base_prompt
        llm_time = 0.0
        attempts = 0
        actions_text = ""
        errors = []

        while attempts < max_retries:
            attempts += 1
            self.get_logger().info(f"Generating action blocks (attempt {attempts}/{max_retries})...")
            t_llm = time.perf_counter()
            try:
                raw_actions = self.call_llm(backend, openai_model, ollama_model, prompt)
            except Exception as e:
                response.success = False
                response.message = f"Action generation failed ({backend}): {e}"
                self.get_logger().error(response.message)
                return response
            llm_time += time.perf_counter() - t_llm

            actions_text = self.extract_action_blocks(raw_actions)
            errors = self.validate_actions(actions_text, requested_names, types_declared, preds_declared)
            if not errors:
                break

            for err in errors:
                self.get_logger().warn(f"Validation: {err}")
            error_list = "\n".join(f"- {e}" for e in errors)
            prompt = (
                f"{base_prompt}\n\n"
                f"--- your previous answer ---\n{raw_actions}\n--- end ---\n\n"
                f"--- errors in your previous answer ---\n{error_list}\n--- end ---\n\n"
                "Rewrite ALL the action blocks and fix every error listed above. "
                "Use ONLY the declared types and predicates."
            )

        self.get_logger().info(f"LLM call time: {llm_time:.2f}s over {attempts} attempt(s)")
        self.log_timing(output_file, llm_time, attempts)

        if errors:
            response.success = False
            response.message = f"Domain rejected after {attempts} attempts: " + "; ".join(errors)
            self.get_logger().error(response.message)
            return response

        out_text = (
            "(define (domain domain_pddl)\n"
            "    (:requirements :negative-preconditions :typing)\n"
            "    (:types\n"
            f"        {types_indented}\n"
            "    )\n"
            "    (:predicates\n"
            f"        {predicates_block}\n"
            "    )\n\n"
            "    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; ACTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\n\n"
            f"    {actions_text}\n"
            ")\n"
        )

        print("=" * 60)
        print("GENERATED DOMAIN:")
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
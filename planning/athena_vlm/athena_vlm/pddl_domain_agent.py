#!/usr/bin/env python3

import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import yaml
from openai import OpenAI


def read_text(p: str) -> str:
    return Path(p).read_text(encoding="utf-8")


class YamlToPddlAgentNode(Node):
    def __init__(self):
        super().__init__("pddl_domain_agent_node")

        self.declare_parameter("robot_yaml_path", "")
        self.declare_parameter("base_domain_path", "")
        self.declare_parameter("output_domain_path", "")

        self.pub = self.create_publisher(String, "/generated_domain", 10)

        self.run_once()

    def run_once(self):
        robot_yaml_path = str(self.get_parameter("robot_yaml_path").value)
        base_domain_path = str(self.get_parameter("base_domain_path").value)
        output_domain_path = str(self.get_parameter("output_domain_path").value)

        if not robot_yaml_path or not base_domain_path or not output_domain_path:
            self.get_logger().error("Missing params: robot_yaml_path, base_domain_path, output_domain_path")
            return

        if not os.environ.get("OPENAI_API_KEY"):
            self.get_logger().error("OPENAI_API_KEY not set")
            return

        try:
            robot_cfg = yaml.safe_load(read_text(robot_yaml_path))
            base_domain = read_text(base_domain_path)
        except Exception as e:
            self.get_logger().error(f"File read error: {e}")
            return

        prompt = f"""
You are a robotics + PDDL engineer.

Generate a NEW full PDDL domain file.

Constraints:
- Keep all existing actions EXACTLY as they are.
- Only add new predicates/types/actions if justified by robot_parameters.yaml.
- If capabilities.manipulation is false → do NOT add load/unload.
- STRIPS only.

Return ONLY the full domain file text.

--- robot_parameters.yaml ---
{yaml.safe_dump(robot_cfg, sort_keys=False).strip()}
--- end ---

--- base_domain.pddl ---
{base_domain.strip()}
--- end ---
""".strip()

        try:
            client = OpenAI()
            resp = client.responses.create(
                model="gpt-5.2",
                input=prompt,
            )
            out_text = resp.output_text.strip()
        except Exception as e:
            self.get_logger().error(f"LLM call failed: {e}")
            return

        # Save to file
        try:
            out_path = Path(output_domain_path)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(out_text, encoding="utf-8")
            self.get_logger().info(f"Saved domain: {output_domain_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write file: {e}")
            return

        print(out_text)

        msg = String()
        msg.data = out_text
        self.pub.publish(msg)

        self.get_logger().info("Published /generated_domain")



def main():
    rclpy.init()
    node = YamlToPddlAgentNode()

    rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
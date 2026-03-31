#!/usr/bin/env python3
import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


# =========================
# DATA STRUCTURES
# =========================

class Joint:
    def __init__(self, name, jtype, parent, child, mimic=None):
        self.name = name
        self.jtype = jtype
        self.parent = parent
        self.child = child
        self.mimic = mimic


class URDFFacts:
    def __init__(self, robot_name, links, joints):
        self.robot_name = robot_name
        self.links = links
        self.joints = joints


# =========================
# URDF PARSER
# =========================

def parse_urdf(path: str) -> URDFFacts:
    tree = ET.parse(path)
    root = tree.getroot()

    robot_name = root.attrib.get("name", "robot")
    links = set()
    joints = {}

    for link in root.findall("link"):
        links.add(link.attrib["name"])

    for j in root.findall("joint"):
        name = j.attrib["name"]
        jtype = j.attrib["type"]
        parent = j.find("parent").attrib["link"]
        child = j.find("child").attrib["link"]

        mimic_el = j.find("mimic")
        mimic = mimic_el.attrib["joint"] if mimic_el is not None else None

        joints[name] = Joint(name, jtype, parent, child, mimic)

    return URDFFacts(robot_name, links, joints)


# =========================
# HEURISTICS
# =========================

def joint_is_wheel_like(j: Joint) -> bool:
    return "wheel" in j.name.lower()

def joint_is_gripper_like(j: Joint) -> bool:
    n = j.name.lower()
    return any(x in n for x in ["gripper", "finger", "hand", "jaw"])

def is_non_fixed_joint(j: Joint) -> bool:
    return j.jtype in ["revolute", "continuous", "prismatic"]

def is_arm_chain_joint(j: Joint) -> bool:
    return is_non_fixed_joint(j) and not joint_is_wheel_like(j)


# =========================
# LLM AGENT (AGENT 1)
# =========================

def call_llm_reasoner(facts: URDFFacts, model: str = "gpt-4.1-mini") -> Optional[dict]:
    """
    Uses LLM if OpenAI client is available AND OPENAI_API_KEY is set.
    Otherwise returns None and rule-based is used.
    """
    if not os.environ.get("OPENAI_API_KEY"):
        return None

    try:
        from openai import OpenAI
        client = OpenAI()
    except Exception:
        return None

    joints_summary = []
    for j in facts.joints.values():
        joints_summary.append({
            "name": j.name,
            "type": j.jtype,
            "parent": j.parent,
            "child": j.child,
            "mimic": j.mimic
        })

    prompt = {
        "robot_name": facts.robot_name,
        "joints": joints_summary,
        "links": list(facts.links)
    }

    system = (
        "You extract robot capabilities from URDF facts (links + joints).\n"
        "Return JSON ONLY (no markdown, no explanations).\n"
        "You MUST follow this exact schema and types:\n"
        "{\n"
        '  "robot": {"name": string},\n'
        '  "capabilities": {"navigation": boolean, "manipulation": boolean},\n'
        '  "evidence": {\n'
        '    "navigation": {"wheel_joints": [string], "base_link_guess": string},\n'
        '    "manipulation": {"arm_joints": [string], "gripper_joints": [string], "ee_link_guess": string}\n'
        "  },\n"
        '  "confidence": {"navigation": "low|medium|high", "manipulation": "low|medium|high"},\n'
        '  "context": {}\n'
        "}\n"
        "Rules:\n"
        "- evidence.navigation.wheel_joints MUST list the names of joints that are wheels.\n"
        '  A wheel joint is a joint whose name contains "wheel" (case-insensitive) and type is revolute or continuous.\n'
        "- Set capabilities.navigation=true iff wheel_joints has length >= 2.\n"
        "- evidence.manipulation.gripper_joints MUST list joints whose name contains any of: gripper, finger, hand, jaw.\n"
        "- evidence.manipulation.arm_joints MUST list non-fixed joints that are NOT wheel joints.\n"
        "- Set capabilities.manipulation=true iff arm_joints length >= 2 AND gripper_joints length >= 1.\n"
        '- If manipulation is false, set ee_link_guess="" (empty string).\n'
        "- Use ONLY joint/link names provided in the input. Do not invent names.\n"
    )

    # NOTE: uses chat.completions like your original script
    try:
        resp = client.chat.completions.create(
            model=model,
            temperature=0,
            messages=[
                {"role": "system", "content": system},
                {"role": "user", "content": json.dumps(prompt)}
            ],
        )
        return json.loads(resp.choices[0].message.content)
    except Exception:
        return None


# =========================
# RULE BASED FALLBACK
# =========================

def rule_based_reasoner(facts: URDFFacts) -> dict:
    wheel_joints = []
    for j in facts.joints.values():
        if joint_is_wheel_like(j) and is_non_fixed_joint(j):
            wheel_joints.append(j.name)

    nav = len(wheel_joints) >= 2

    arm_joints = []
    for j in facts.joints.values():
        if is_arm_chain_joint(j):
            arm_joints.append(j.name)

    gripper_joints = []
    for j in facts.joints.values():
        if joint_is_gripper_like(j) and is_non_fixed_joint(j):
            gripper_joints.append(j.name)

    manip = len(arm_joints) >= 2 and len(gripper_joints) >= 1

    return {
        "robot": {"name": facts.robot_name},
        "capabilities": {"navigation": nav, "manipulation": manip},
        "evidence": {
            "navigation": {"wheel_joints": wheel_joints, "base_link_guess": ""},
            "manipulation": {"arm_joints": arm_joints, "gripper_joints": gripper_joints, "ee_link_guess": ""}
        },
        "confidence": {
            "navigation": "high" if nav else "low",
            "manipulation": "high" if manip else "low"
        },
        "context": {}
    }


# =========================
# CHECKER (AGENT 2)
# =========================

def checker_validate_and_correct(facts: URDFFacts, y: dict) -> Tuple[dict, List[str]]:
    issues = []

    if not isinstance(y, dict):
        y = {}

    for k in ["robot", "capabilities", "evidence", "confidence", "context"]:
        y.setdefault(k, {})

    if not isinstance(y["evidence"], dict):
        y["evidence"] = {}
    y["evidence"].setdefault("navigation", {})
    y["evidence"].setdefault("manipulation", {})

    if not isinstance(y["confidence"], dict):
        y["confidence"] = {}
    if not isinstance(y["capabilities"], dict):
        y["capabilities"] = {}

    # ---- navigation ----
    nav_ev = y["evidence"]["navigation"]
    wheel_joints = nav_ev.get("wheel_joints", []) or []
    base_guess = nav_ev.get("base_link_guess", "") or ""

    if not wheel_joints:
        for j in facts.joints.values():
            if j.jtype in ["revolute", "continuous"]:
                child = j.child.lower()
                name = j.name.lower()
                if ("wheel" in name) or ("wheel" in child) or ("tire" in name) or ("tire" in child):
                    wheel_joints.append(j.name)

    valid_wheels = []
    for jn in wheel_joints:
        if jn in facts.joints:
            j = facts.joints[jn]
            if j.jtype in ["revolute", "continuous"] and joint_is_wheel_like(j):
                valid_wheels.append(jn)

    nav_supported = len(set(valid_wheels)) >= 2
    if not nav_supported:
        base_guess = ""

    y["evidence"]["navigation"]["wheel_joints"] = valid_wheels
    y["evidence"]["navigation"]["base_link_guess"] = base_guess
    y["capabilities"]["navigation"] = nav_supported
    y["confidence"]["navigation"] = "high" if nav_supported else "low"

    # ---- manipulation ----
    manip_ev = y["evidence"]["manipulation"]
    arm_joints = manip_ev.get("arm_joints", []) or []
    gripper_joints = manip_ev.get("gripper_joints", []) or []
    ee_guess = manip_ev.get("ee_link_guess", "") or ""

    valid_arm = [j for j in arm_joints if j in facts.joints and is_arm_chain_joint(facts.joints[j])]
    valid_grip = [j for j in gripper_joints if j in facts.joints and joint_is_gripper_like(facts.joints[j])]

    if len(valid_arm) < 2:
        if len(valid_grip) > 0:
            issues.append("Manipulator incomplete: gripper but no arm chain")
        elif len(arm_joints) >= 2:
            issues.append("Manipulator incomplete: arm joints but invalid chain")
        valid_arm = []
        valid_grip = []
        ee_guess = ""

    manip_supported = len(valid_arm) >= 2 and len(valid_grip) >= 1
    if not manip_supported:
        ee_guess = ""

    y["evidence"]["manipulation"]["arm_joints"] = valid_arm
    y["evidence"]["manipulation"]["gripper_joints"] = valid_grip
    y["evidence"]["manipulation"]["ee_link_guess"] = ee_guess

    y["capabilities"]["manipulation"] = manip_supported
    y["confidence"]["manipulation"] = "high" if manip_supported else "low"

    return y, issues


# =========================
# ROS2 NODE
# =========================

class UrdCapabilitiesAgentNode(Node):
    def __init__(self):
        super().__init__("capability_extractor_agent_node")

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("out_yaml_path", "")

        self.run_pipeline()

    def run_pipeline(self):
        urdf_path = self.get_parameter("urdf_path").value
        out_yaml = self.get_parameter("out_yaml_path").value

        if not urdf_path or not out_yaml:
            self.get_logger().error("Missing params")
            return

        facts = parse_urdf(urdf_path)

        y = call_llm_reasoner(facts)
        if y is None:
            y = rule_based_reasoner(facts)

        y, issues = checker_validate_and_correct(facts, y)

        with open(out_yaml, "w") as f:
            yaml.safe_dump(y, f, sort_keys=False)

        self.get_logger().info(f"Saved: {out_yaml}")


def main():
    rclpy.init()
    node = UrdCapabilitiesAgentNode()

    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
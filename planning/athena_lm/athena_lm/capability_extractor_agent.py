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


SENSOR_KEYWORDS = ["camera", "lidar", "laser", "depth", "rgbd", "imu", "sonar", "sensor", "kinect", "realsense"]
GRIPPER_KEYWORDS = ["gripper", "finger", "jaw", "claw"]
HAND_KEYWORDS = ["hand", "palm", "wrist"]

WHEEL_KEYWORDS = ["wheel", "tire"]
LEG_KEYWORDS = ["leg", "hip", "thigh", "knee", "ankle", "foot"]


def joint_is_wheel_like(j: Joint) -> bool:
    n = j.name.lower()
    c = j.child.lower()
    return any(k in n or k in c for k in WHEEL_KEYWORDS)


def joint_is_leg_like(j: Joint) -> bool:
    n = j.name.lower()
    c = j.child.lower()
    return any(k in n or k in c for k in LEG_KEYWORDS)


def joint_is_locomotion_like(j: Joint) -> bool:
    return joint_is_wheel_like(j) or joint_is_leg_like(j)


def joint_is_gripper_like(j: Joint) -> bool:
    n = j.name.lower()
    return any(k in n for k in GRIPPER_KEYWORDS)


def link_is_hand_like(link_name: str) -> bool:
    n = link_name.lower()
    return any(k in n for k in HAND_KEYWORDS)


def link_is_sensor_like(link_name: str) -> bool:
    n = link_name.lower()
    return any(k in n for k in SENSOR_KEYWORDS)


def is_non_fixed_joint(j: Joint) -> bool:
    return j.jtype in ["revolute", "continuous", "prismatic"]


def is_arm_chain_joint(j: Joint) -> bool:
    return is_non_fixed_joint(j) and not joint_is_locomotion_like(j) and not joint_is_gripper_like(j)


def wheel_joints(facts: URDFFacts) -> List[str]:
    return [
        j.name for j in facts.joints.values()
        if joint_is_wheel_like(j) and j.jtype in ["revolute", "continuous"]
    ]


def leg_joints(facts: URDFFacts) -> List[str]:
    return [
        j.name for j in facts.joints.values()
        if joint_is_leg_like(j) and j.jtype in ["revolute", "continuous", "prismatic"]
    ]


def call_llm_reasoner(facts: URDFFacts, model: str = "gpt-4.1-mini") -> Optional[dict]:
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
        '  "capabilities": {\n'
        '    "navigation": boolean,\n'
        '    "legged": boolean,\n'
        '    "perception": boolean,\n'
        '    "manipulation": boolean,\n'
        '    "grasping": boolean\n'
        '  }\n'
        "}\n"
        "Rules:\n"
        "- navigation = true iff there are at least 2 wheel joints (name or child link contains 'wheel' or 'tire'; joint type revolute or continuous).\n"
        "- legged = true iff there are at least 2 leg joints (name or child link contains 'leg', 'hip', 'thigh', 'knee', 'ankle', or 'foot'; joint type revolute, continuous, or prismatic).\n"
        "- grasping = true iff there is at least one joint whose name contains: gripper, finger, jaw, or claw (type revolute, continuous, or prismatic).\n"
        "- manipulation = true iff there is at least one link whose name contains: hand, palm, or wrist, AND at least 2 non-fixed joints that are not locomotion (wheel or leg) and not gripper.\n"
        "- perception = true iff there is at least one link whose name contains any of: camera, lidar, laser, depth, rgbd, imu, sonar, sensor, kinect, realsense.\n"
        "- Use ONLY joint/link names provided in the input. Do not invent names.\n"
    )

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


def rule_based_reasoner(facts: URDFFacts) -> dict:
    nav = len(wheel_joints(facts)) >= 2

    legged = len(leg_joints(facts)) >= 2

    gripper_joints = [
        j.name for j in facts.joints.values()
        if joint_is_gripper_like(j) and is_non_fixed_joint(j)
    ]
    grasping = len(gripper_joints) >= 1

    arm_joints = [
        j.name for j in facts.joints.values()
        if is_arm_chain_joint(j)
    ]
    hand_links = [l for l in facts.links if link_is_hand_like(l)]
    manip = len(arm_joints) >= 2 and len(hand_links) >= 1

    sensor_links = [l for l in facts.links if link_is_sensor_like(l)]
    perception = len(sensor_links) >= 1

    return {
        "robot": {"name": facts.robot_name},
        "capabilities": {
            "navigation": nav,
            "legged": legged,
            "perception": perception,
            "manipulation": manip,
            "grasping": grasping
        }
    }


def checker_validate_and_correct(facts: URDFFacts, y: dict) -> Tuple[dict, List[str]]:
    issues = []

    if not isinstance(y, dict):
        y = {}

    y.setdefault("robot", {})
    if not isinstance(y["robot"], dict):
        y["robot"] = {}
    y["robot"].setdefault("name", facts.robot_name)

    y.setdefault("capabilities", {})
    if not isinstance(y["capabilities"], dict):
        y["capabilities"] = {}

    caps = y["capabilities"]

    nav_truth = len(wheel_joints(facts)) >= 2
    if caps.get("navigation") != nav_truth:
        issues.append(f"navigation corrected to {nav_truth}")
    caps["navigation"] = nav_truth

    legged_truth = len(leg_joints(facts)) >= 2
    if caps.get("legged") != legged_truth:
        issues.append(f"legged corrected to {legged_truth}")
    caps["legged"] = legged_truth

    gripper_joints = [
        j.name for j in facts.joints.values()
        if joint_is_gripper_like(j) and is_non_fixed_joint(j)
    ]
    grasp_truth = len(gripper_joints) >= 1
    if caps.get("grasping") != grasp_truth:
        issues.append(f"grasping corrected to {grasp_truth}")
    caps["grasping"] = grasp_truth

    arm_joints = [
        j.name for j in facts.joints.values()
        if is_arm_chain_joint(j)
    ]
    hand_links = [l for l in facts.links if link_is_hand_like(l)]
    manip_truth = len(arm_joints) >= 2 and len(hand_links) >= 1
    if caps.get("manipulation") != manip_truth:
        issues.append(f"manipulation corrected to {manip_truth}")
    caps["manipulation"] = manip_truth

    sensor_links = [l for l in facts.links if link_is_sensor_like(l)]
    perc_truth = len(sensor_links) >= 1
    if caps.get("perception") != perc_truth:
        issues.append(f"perception corrected to {perc_truth}")
    caps["perception"] = perc_truth

    ordered_caps = {
        "navigation": caps["navigation"],
        "legged": caps["legged"],
        "perception": caps["perception"],
        "manipulation": caps["manipulation"],
        "grasping": caps["grasping"]
    }
    y["capabilities"] = ordered_caps

    return y, issues


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

        for issue in issues:
            self.get_logger().warn(issue)

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
#!/usr/bin/env python3
"""Publish a message on the 'instruction' topic, by console or by voice.

Usage:
    python3 publish_instruction.py            # console mode (default)
    python3 publish_instruction.py voice      # voice mode

Voice mode needs two extra packages:
    sudo apt install portaudio19-dev
    pip install SpeechRecognition pyaudio
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TOPIC_NAME = "instruction"


class InstructionPublisher(Node):
    def __init__(self):
        super().__init__("instruction_publisher")
        self.publisher = self.create_publisher(String, TOPIC_NAME, 10)

    def publish(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info('Published: "%s"' % text)


def run_console(node):
    print('Console mode. Type an instruction and press Enter.')
    print('Type "quit" to exit.')
    while rclpy.ok():
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if text.lower() in ("quit", "exit", "q"):
            break
        if text:
            node.publish(text)


def run_voice(node):
    try:
        import speech_recognition as sr
    except ImportError:
        print("Voice mode needs the SpeechRecognition package.")
        print("Install it with: pip install SpeechRecognition pyaudio")
        return

    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    print("Voice mode. Adjusting for background noise, please wait...")
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
    print("Ready. Speak your instruction. Press Ctrl+C to exit.")

    while rclpy.ok():
        try:
            print("Listening...")
            with microphone as source:
                audio = recognizer.listen(source)
            text = recognizer.recognize_google(audio)
            print('Heard: "%s"' % text)
            node.publish(text)
        except sr.UnknownValueError:
            print("Could not understand the audio, please try again.")
        except KeyboardInterrupt:
            break
        except Exception as error:
            print("Error: %s" % error)


def main():
    rclpy.init()
    node = InstructionPublisher()

    mode = sys.argv[1] if len(sys.argv) > 1 else "console"

    try:
        if mode == "voice":
            run_voice(node)
        else:
            run_console(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
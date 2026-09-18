---
sidebar_position: 2
title: Voice-to-Action with OpenAI Whisper
---

# Voice-to-Action with OpenAI Whisper

## Learning Objectives
- Learn the basics of speech recognition.
- Integrate Whisper for voice commands.
- Convert audio into actionable text for the robot.

## What is OpenAI Whisper?
Whisper is a powerful Automatic Speech Recognition (ASR) model capable of converting audio into text with high accuracy.

## Integrating Whisper with ROS 2
You can build a ROS 2 node that captures microphone audio, sends it to a Whisper API or local model, and publishes the transcribed text to a `/voice_command` topic.

## Code Example
```python
# Simple command processing snippet
import whisper

model = whisper.load_model("base")
audio = "command.wav"
result = model.transcribe(audio)
command = result["text"]
# Now publish 'command' to a ROS 2 topic
```

## Takeaways
- Speech recognition provides a natural human-robot interface.
- Transcribing audio is the first step in translating intent to action.

## Review Questions
1. Why is an ASR model like Whisper critical for VLA?

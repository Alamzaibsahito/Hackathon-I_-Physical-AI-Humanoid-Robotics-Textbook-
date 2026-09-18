---
sidebar_position: 3
title: Cognitive Planning with LLMs
---

# Cognitive Planning with LLMs

## Learning Objectives
- Learn how LLMs decompose complex natural language commands.
- Understand prompt engineering for robotic task planning.
- Create a sequence of actionable ROS 2 commands.

## Task Decomposition
LLMs are excellent at taking high-level goals (e.g., "Bring me the water") and decomposing them into sub-goals:
1. `Find_Water`
2. `Navigate_To_Water`
3. `Grasp_Water`
4. `Return_To_User`

## Prompt Engineering
The key is to instruct the LLM to output actions in a structured, machine-readable format (e.g., JSON) that your ROS 2 node can easily parse.

## Takeaways
- LLMs provide the reasoning layer between human intent and robotic execution.
- Structural prompting is essential for reliable task execution.

## Review Questions
1. How does an LLM help in complex robotic task planning?

# Hackathon Book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-10

## Active Technologies

- **Markdown (Docusaurus)** (overall-project)
- **Python 3.9+** (for code examples) (overall-project)
- **Docusaurus** (overall-project)
- ROS 2 Humble (001-book-module1-ros2, 003-isaac-robot-brain-module3, 001-vla-robotics-module)
- rclpy (001-book-module1-ros2, 003-isaac-robot-brain-module3, 001-vla-robotics-module)
- NVIDIA Isaac Sim (003-isaac-robot-brain-module3)
- NVIDIA Isaac ROS (003-isaac-robot-brain-module3)
- Nav2 (003-isaac-robot-brain-module3, 001-vla-robotics-module)
- Whisper (001-vla-robotics-module)
- Vision-Language Models (VLMs) (001-vla-robotics-module)
- ROS 2 Actions (001-vla-robotics-module)
- Behavior Trees (001-vla-robotics-module)

## Project Structure

```text
docs/
└── module1-ros2/
    ├── _category_.json
    ├── chapter1-ros2-fundamentals.md
    ├── chapter2-python-agent-bridges.md
    └── chapter3-humanoid-models-urdf.md
```

## Commands

Python: cd src; pytest; ruff check .
# Docusaurus commands for building/testing would go here if specified.

## Code Style

Python: Follow standard conventions
Markdown: Follow Docusaurus conventions

## Recent Changes

- **overall-project: Adopted Docusaurus, Markdown, and Python for the book project.**
- 001-book-module1-ros2: Added Markdown (Docusaurus) + Python 3.9+ (for code examples) + Docusaurus, ROS 2 Humble, rclpy
- 001-vla-robotics-module: Added Whisper, VLMs, ROS 2 Actions, Behavior Trees, ROS 2 Humble, rclpy, Nav2.

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->

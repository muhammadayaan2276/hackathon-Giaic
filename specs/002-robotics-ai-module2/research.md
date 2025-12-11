# Research Findings for Book Module 2 – Robotics AI & Control

## Phase 0: Outline & Research Findings

This document consolidates research findings for Phase 0, addressing foundational elements for Module 2.

### Research AI Decision Pipelines

**Decision**: Focus on conceptual understanding of AI decision-making pipelines in ROS 2. Emphasize standard patterns like state machines, behavior trees (conceptually), and reactive approaches. Avoid deep dives into specific AI algorithms (e.g., Reinforcement Learning) as per constraints.

**Rationale**: This aligns with the "beginner AI developers" audience and the constraint against "deep RL tutorials" or "unrelated theory." The focus will be on *how* AI makes decisions within a robotic context using ROS 2.

**Alternatives Considered**:
*   **In-depth algorithm explanations**: Rejected due to audience and constraint against "unrelated theory."
*   **Specific AI framework integration**: Rejected as it might introduce vendor lock-in or be too advanced for beginners. Focus on ROS 2 integration patterns.

### Research Sensor Fusion & Perception

**Decision**: Cover fundamental sensor fusion techniques (e.g., Kalman filters conceptually) and common perception tasks (object detection, localization) using ROS 2. Prioritize practical ROS 2 Python examples.

**Rationale**: These are core concepts for robots to understand their environment, fitting the audience and topic. Using ROS 2 tools makes it practical for students.

**Alternatives Considered**:
*   **Advanced SLAM algorithms**: Rejected due to beginner audience and timeline. Focus on conceptual understanding.
*   **Proprietary sensor fusion methods**: Rejected due to constraint against "vendor/hardware guides."

### Research Motion Planning Basics

**Decision**: Introduce fundamental motion planning concepts (e.g., pathfinding, obstacle avoidance) and conceptual overview of ROS 2 tools like MoveIt! (without deep implementation details).

**Rationale**: Provides an introductory understanding without overwhelming beginners, aligning with the "beginner AI developers" audience.

**Alternatives Considered**:
*   **Detailed algorithm implementations**: Rejected due to audience and beginner focus.
*   **In-depth configuration of planning libraries**: Rejected as it goes beyond basic concepts.

### Identify ROS 2 Python Examples

**Decision**: Provide clear, reproducible Python `rclpy` examples for each chapter that demonstrate AI integration, perception data handling, and basic control commands.

**Rationale**: Practical code examples are essential for the target audience and for the success criteria.

**Alternatives Considered**:
*   **C++ examples**: Rejected as Python is specified for AI integration and is generally more accessible for beginners.
*   **Complex, multi-node examples**: Rejected to maintain clarity and focus for beginners.

### Research Diagramming Tools

**Decision**: Primarily use Mermaid.js for diagrams, as it's compatible with Docusaurus and allows for text-based diagram creation, fitting a "docs-as-code" approach.

**Rationale**: Native Docusaurus support and ease of integration make Mermaid.js the most efficient choice for generating diagrams within markdown.

**Alternatives Considered**:
*   **External image files**: Less flexible for updates and version control.
*   **JavaScript charting libraries**: Potentially overkill for conceptual diagrams.

# Chapter 2: Cognitive Planning with LLMs

## Introduction

Intelligent robots must interpret high-level human goals and generate complex action sequences. Cognitive planning, enhanced by Large Language Models (LLMs), bridges abstract natural language instructions (e.g., "Clean the room") with discrete, structured robot actions. This chapter explores how LLMs transform human intent into multi-step ROS 2 action plans.

## LLMs for Transforming Natural Language to Robot Plans

LLMs leverage their reasoning to act as high-level robotic planners, decomposing general instructions into executable actions.

### The LLM Planning Process (Conceptual)

1.  **Intent Reception**: LLM receives natural language command.
2.  **Contextual Understanding**: LLM analyzes command within robot's context (capabilities, environment).
3.  **Task Decomposition**: High-level goals are broken into atomic actions, e.g., "Clean the room" becomes: `navigate_to_area("living_room")`, `find_object("red_ball")`, `pick_up_object("red_ball")`, etc.
4.  **Action Parameterization**: LLM identifies necessary parameters (e.g., "red_ball").
5.  **ROS 2 Plan Generation**: Decomposed actions are formatted into a structured plan (sequence of ROS 2 Action Goals).

## Step-by-Step Flows for LLM Planning

Consider: "Please pick up the empty cup from the table and throw it in the trash."

### LLM Planning Workflow (Conceptual Diagram)

```mermaid
graph TD
    A[Natural Language Command: "Pick up empty cup and throw in trash"] --> B{LLM Interpretation & Decomposition};
    B --> C{Action Sequence Generation};
    C -- Action: `navigate_to_object("empty_cup")` --> D1[ROS 2 Action Goal 1];
    C -- Action: `pick_up_object("empty_cup")` --> D2[ROS 2 Action Goal 2];
    C -- Action: `navigate_to_object("trash_can")` --> D3[ROS 2 Action Goal 3];
    C -- Action: `place_object("empty_cup", "trash_can")` --> D4[ROS 2 Action Goal 4];
    D1 --> E[ROS 2 Action Server];
    D2 --> E;
    D3 --> E;
    D4 --> E;
    E --> F[Robot Executes Plan];
```
This diagram illustrates the flow from a natural language command to a robot executing a generated plan via ROS 2 Action Goals.

## Realistic ROS 2 Action-Style Examples for LLM-Generated Plans (Conceptual)

Extending from Chapter 1's `PickUp` action, we introduce `NavigateTo` and `PlaceObject`.

### `NavigateTo.action` Definition (Conceptual)

```
# Goal
string target_location_name # e.g., "kitchen"
geometry_msgs/PoseStamped target_pose # Optional specific pose

---
# Result
bool success      # True if navigation was successful
string message    # Outcome message

---
# Feedback
float32 distance_to_goal
string current_status
```

### `PlaceObject.action` Definition (Conceptual)

```
# Goal
string object_id          # Object being held
string target_container_name # e.g., "basket"
geometry_msgs/PoseStamped target_pose # Optional specific placement pose

---
# Result
bool success      # True if placement was successful
string message    # Outcome message

---
# Feedback
float32 progress  # % completed
string status     # Current status
```

### LLM-Generated Plan (Conceptual Python/rclpy)

An LLM generates a structured plan (e.g., JSON) interpreted by a control script.

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# Assuming custom action definitions from example_interfaces.action

class LLMPlanExecutor(Node):
    def __init__(self):
        super().__init__('llm_plan_executor')
        self.pickup_client = ActionClient(self, PickUp, 'pick_up_robot_action')
        self.navigate_client = ActionClient(self, NavigateTo, 'navigate_to_robot_action')
        self.place_client = ActionClient(self, PlaceObject, 'place_object_robot_action')
        # Wait for action servers

    async def execute_plan(self):
        plan = [ # Simplified LLM plan representation
            {"action": "navigate_to", "target": "empty_cup_location"},
            {"action": "pick_up", "object_id": "empty_cup"},
            {"action": "navigate_to", "target": "trash_can_location"},
            {"action": "place_object", "object_id": "empty_cup", "target_container": "trash_can"}
        ]
        # Iterate and send goals, handle results (simplified for brevity)
        return True

def main(args=None):
    rclpy.init(args=args)
    executor = LLMPlanExecutor()
    rclpy.spin_until_future_complete(executor, executor.execute_plan()) 
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Note: This example is conceptual; requires custom ROS 2 action definitions and full async implementation.*

## Robot Behavior Reasoning and Error Handling in LLM Planning

LLMs aid in robust robotic planning by:
*   **Pre-computation of Failure Modes**: Predicting issues (e.g., "cup too heavy").
*   **Adaptive Planning**: Generating alternative strategies if actions fail.
*   **Human-in-the-Loop Interaction**: Formulating questions to users for unresolved issues.

This integration, often with behavior trees, enables dynamic and resilient robot operation.

## Conclusion

LLMs empower robots with cognitive planning, enabling understanding and execution of complex natural language instructions. By decomposing tasks and generating structured action sequences, LLMs increase humanoid robot autonomy and adaptability, paving the way for complete VLA integration.
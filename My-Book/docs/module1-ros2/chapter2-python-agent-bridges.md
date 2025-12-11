---
sidebar_position: 2
---

# Chapter 2: Python Agent Bridges

In the previous chapter, we explored the fundamental communication concepts of ROS 2: Nodes, Topics, and Services. Now, we will bridge the gap between the ROS 2 world and the world of Artificial Intelligence by using Python. Most AI and machine learning frameworks are either written in or have excellent support for Python, making it the de facto language for AI development.

This chapter focuses on `rclpy` (ROS Client Library for Python), the official Python library for interfacing with ROS 2. We will learn how to write ROS 2 nodes in Python, enabling you to integrate your AI logic, machine learning models, and any other Python-based agent directly into a robotic system.

## Introduction to `rclpy`

`rclpy` is more than just a simple wrapper. It is a Pythonic client library that provides access to all the core ROS 2 functionalities. It allows you to:

-   Create ROS 2 nodes.
-   Publish and subscribe to topics.
-   Create and call services.
-   Manage parameters.
-   And much more.

By using `rclpy`, your Python code becomes a first-class citizen in the ROS 2 ecosystem, able to communicate seamlessly with other nodes written in C++, Java, or any other ROS 2-supported language.

## Creating a ROS 2 Node in Python

Everything in ROS 2 starts with a node. A Python-based ROS 2 node is simply a Python class that inherits from `rclpy.node.Node`. Let's break down the essential structure.

### The Basic Structure

A minimal `rclpy` node looks like this:

```python
import rclpy
from rclpy.node import Node

class MyPythonNode(Node):
    def __init__(self):
        # Call the constructor of the parent class (Node)
        # and give the node a unique name.
        super().__init__('my_python_node')
        self.get_logger().info('My Python Node has started!')

def main(args=None):
    # 1. Initialize the rclpy library
    rclpy.init(args=args)

    # 2. Create an instance of your node
    my_node = MyPythonNode()

    # 3. "Spin" the node, making it available to the ROS 2 network
    #    and keeping it running until it's shut down (e.g., with Ctrl+C).
    rclpy.spin(my_node)

    # 4. (Optional but good practice) Destroy the node and shutdown rclpy
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's dissect the `main` function:
1.  `rclpy.init()`: This is the first thing you must do. It initializes the ROS 2 communication middleware.
2.  `MyPythonNode()`: This creates an instance of our node class.
3.  `rclpy.spin(my_node)`: This is the workhorse. It enters a loop that processes all the node's callbacks (like subscriber callbacks, timer callbacks, etc.). The program will stay in this loop until the node is shut down.
4.  `destroy_node()` and `shutdown()`: These are cleanup functions to gracefully release the resources used by the node and the `rclpy` library.

## Publishing and Subscribing to Topics with `rclpy`

The most common way for an AI agent to interact with a robot is by subscribing to sensor data and publishing control commands. Let's create a simple agent that simulates this behavior.

Imagine an AI agent that "sees" the state of a simple robot (e.g., its position) and tells it where to go next.

-   The robot's state will be published on a `/robot/state` topic.
-   Our AI agent will subscribe to `/robot/state`.
-   After processing the state, the agent will publish a new target position to a `/robot/target` topic.

### A Complete Example: The "Greedy" AI Agent

This example will consist of two nodes:
1.  `RobotSimulatorNode`: A simple node that simulates a robot by publishing its current position and subscribing to target commands.
2.  `GreedyAgentNode`: Our "AI" agent. It subscribes to the robot's position and tells it to move to the next position in a predefined list.

#### The Robot Simulator Node

This node simulates the robot's state and reacts to target commands.

```python
# robot_simulator_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Using a simple integer for position

class RobotSimulatorNode(Node):
    def __init__(self):
        # Initialize node with name 'robot_simulator'
        super().__init__('robot_simulator')
        self.current_position_ = 0  # Starting position
        
        # Publisher for the robot's current state
        # Publishes Int32 messages to the 'robot/state' topic
        self.state_publisher_ = self.create_publisher(Int32, 'robot/state', 10)
        
        # Subscriber for the robot's target commands
        # Subscribes to 'robot/target' topic, calls target_callback on message arrival
        self.target_subscriber_ = self.create_subscription(
            Int32,
            'robot/target',
            self.target_callback,
            10)
            
        # A timer to publish the state every second, simulating robot movement/reporting
        self.state_timer_ = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('Robot Simulator started. Current position: 0')

    def publish_state(self):
        # Creates an Int32 message with the current position and publishes it
        msg = Int32()
        msg.data = self.current_position_
        self.state_publisher_.publish(msg)
        self.get_logger().info(f'Publishing robot state: {self.current_position_}')

    def target_callback(self, msg):
        # This callback is triggered when a new target message is received
        target_position = msg.data
        self.get_logger().info(f'Received new target: {target_position}. Moving...')
        # In a real robot, this would involve planning and moving actuators.
        # Here, we just update the position to simulate reaching the target.
        self.current_position_ = target_position

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulatorNode()
    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Robot simulator stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### The Greedy Agent Node

This node simulates a simple AI agent. It has a list of target positions and commands the robot to move to the next one whenever it receives a state update or issues the first command.

```python
# greedy_agent_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class GreedyAgentNode(Node):
    def __init__(self):
        # Initialize node with name 'greedy_agent'
        super().__init__('greedy_agent')
        # A predefined sequence of target positions for the agent
        self.target_positions_ = [10, 25, 5, 0]
        self.target_index_ = 0  # Index to keep track of the current target
        
        # Publisher for the robot's target commands
        # Publishes Int32 messages to the 'robot/target' topic
        self.target_publisher_ = self.create_publisher(Int32, 'robot/target', 10)
        
        # Subscriber for the robot's current state
        # Subscribes to 'robot/state' topic, calls state_callback on message arrival
        self.state_subscriber_ = self.create_subscription(
            Int32,
            'robot/state',
            self.state_callback,
            10)
            
        self.get_logger().info('Greedy AI Agent started. Ready to issue commands.')

    def state_callback(self, msg):
        # This callback is triggered when the robot's state is updated
        current_position = msg.data
        self.get_logger().info(f'Agent sees robot at position: {current_position}')
        
        # Simple "greedy" logic: if the robot has reached the *previous* target,
        # issue the next command in the sequence.
        # We check against target_index_ - 1 because target_index_ is incremented after issuing command.
        if self.target_index_ > 0 and self.target_index_ <= len(self.target_positions_):
            # If the robot is at the position we *just* commanded it to go to
            if current_position == self.target_positions_[self.target_index_ - 1]:
                 self.issue_new_command()
        elif self.target_index_ == 0: # If this is the first state update, issue the first command
             self.issue_new_command()

    def issue_new_command(self):
        # Checks if there are more targets in the sequence and publishes the next one
        if self.target_index_ < len(self.target_positions_):
            target_msg = Int32()
            target_msg.data = self.target_positions_[self.target_index_]
            self.target_publisher_.publish(target_msg)
            self.get_logger().info(f'Agent issuing new target: {target_msg.data}')
            self.target_index_ += 1 # Move to the next target in the list
        else:
            self.get_logger().info('Agent has completed its sequence of targets.')
            # Optionally, stop the node or publish a completion message
            # rclpy.shutdown() # uncomment to stop the node after completion


def main(args=None):
    rclpy.init(args=args)
    node = GreedyAgentNode()
    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Greedy agent stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Example

To run this example, you would typically save each script as a Python file within a ROS 2 package. Then, you would launch each node in separate terminals (after sourcing your ROS 2 environment and your workspace where the package is built).

**Terminal 1 (Robot Simulator):**
```bash
# Navigate to your package directory and run the simulator script
# Replace 'path/to/your/package' with the actual path
cd path/to/your/package/your_package_name
source install/setup.bash # Or setup.zsh
python robot_simulator_node.py 
```

**Terminal 2 (Greedy Agent):**
```bash
# Navigate to your package directory and run the agent script
cd path/to/your/package/your_package_name
source install/setup.bash # Or setup.zsh
python greedy_agent_node.py
```

You would observe the `RobotSimulatorNode` publishing its state, and the `GreedyAgentNode` subscribing to it, then publishing new target commands back to the simulator. This interaction demonstrates a fundamental feedback loop commonly found in robotics systems, where an agent perceives the environment (via subscribed states) and commands actions (via published targets).

This simple pattern is the foundation for integrating more complex AI. Your "agent" could be a sophisticated reinforcement learning model, a classical path planner, or a computer vision system. As long as it can be controlled or provide information via Python scripts, it can be integrated into a ROS 2 system using `rclpy`.

In the next chapter, we will explore how to represent the physical structure of a robot using URDF, which is essential for visualization and simulation.
---
id: ch02-services-actions
title: ROS 2 Services and Actions
sidebar_label: Services & Actions
sidebar_position: 2
---

# ROS 2 Services and Actions

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the request-response pattern of ROS 2 services
- Implement service servers and clients in Python
- Understand the goal-feedback-result pattern of ROS 2 actions
- Implement action servers and clients with cancellation support
- Choose the appropriate communication pattern (topic, service, or action) for different scenarios

## Introduction

In the previous chapter, we learned about ROS 2 nodes and topics, which provide asynchronous, many-to-many communication through the publish-subscribe pattern. However, not all robotics problems fit this pattern. Sometimes we need:

- **Synchronous request-response**: "What is the current battery level?" requires an immediate answer
- **Long-running tasks with feedback**: "Navigate to the kitchen" takes time and we want progress updates
- **Cancellable operations**: "Stop navigating, emergency detected!"

ROS 2 provides two additional communication patterns to address these needs:
- **Services**: Synchronous request-response for short operations
- **Actions**: Asynchronous goal-feedback-result for long-running tasks

Understanding when to use each pattern is crucial for building robust robotic systems.

## 1. ROS 2 Services

### What Are Services?

Services implement a **client-server** model where:
- A **service server** provides functionality and waits for requests
- A **service client** sends a request and blocks until receiving a response
- Communication is **one-to-one** and **synchronous**

**Key characteristics:**
- **Blocking**: Client waits for the response before continuing
- **Short-lived**: Designed for quick operations (typically &lt;1 second)
- **Request-response**: One request produces exactly one response
- **No feedback**: No progress updates during execution

### When to Use Services

Services are ideal for:
- ✅ Querying robot state (battery level, sensor readings)
- ✅ Triggering actions (reset odometry, start/stop recording)
- ✅ Configuration changes (set parameters, update gains)
- ✅ Short computations (coordinate transformations, simple calculations)

Avoid services for:
- ❌ Long-running tasks (navigation, manipulation)
- ❌ Streaming data (use topics instead)
- ❌ Operations that need progress updates
- ❌ Tasks that might need cancellation

### Service Definition

Services are defined using `.srv` files with two parts separated by `---`:

```python
# example_interfaces/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

The upper section defines the **request**, and the lower section defines the **response**.

### Implementing a Service Server

Here's a complete service server that adds two integers:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,           # Service type
            'add_two_ints',       # Service name
            self.add_callback     # Callback function
        )

        self.get_logger().info('Service server ready')

    def add_callback(self, request, response):
        """Handle incoming service requests"""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key points:**
- `create_service()` registers the service with the ROS 2 system
- The callback receives `request` and `response` objects
- The callback must return the filled `response` object
- Service callbacks should be fast (avoid blocking operations)

### Implementing a Service Client

Here's the corresponding client:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        """Send request and wait for response"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service asynchronously
        future = self.cli.call_async(request)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)

        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()

    result = client.send_request(5, 7)
    client.get_logger().info(f'Result: {result.sum}')

    client.destroy_node()
    rclpy.shutdown()
```

**Key points:**
- `wait_for_service()` ensures the server is available before calling
- `call_async()` sends the request and returns a future
- `spin_until_future_complete()` blocks until the response arrives
- Always check if the service call succeeded before using the result

## 2. ROS 2 Actions

### What Are Actions?

Actions provide a **goal-feedback-result** pattern for long-running tasks:
- A **goal** defines what to achieve
- **Feedback** provides progress updates during execution
- A **result** is returned when the task completes
- Goals can be **cancelled** mid-execution

**Key characteristics:**
- **Asynchronous**: Client doesn't block while waiting
- **Long-running**: Designed for tasks taking seconds to minutes
- **Feedback**: Regular progress updates
- **Cancellable**: Can be stopped before completion
- **Stateful**: Tracks goal state (pending, active, succeeded, cancelled, aborted)

### When to Use Actions

Actions are ideal for:
- ✅ Navigation to a goal position
- ✅ Manipulation tasks (pick and place)
- ✅ Trajectory execution
- ✅ Long computations with progress tracking
- ✅ Any task that might need cancellation

Avoid actions for:
- ❌ Quick operations (use services)
- ❌ Continuous data streams (use topics)
- ❌ Simple queries

### Action Definition

Actions are defined using `.action` files with three parts:

```python
# example_interfaces/action/Fibonacci.action

# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

The three sections define:
1. **Goal**: What the client wants to achieve
2. **Result**: What the server returns upon completion
3. **Feedback**: Progress updates sent during execution

### Implementing an Action Server

Here's an action server that computes Fibonacci sequences:

```python
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Action server ready')

    def goal_callback(self, goal_request):
        """Accept or reject goals"""
        if goal_request.order < 0 or goal_request.order > 50:
            self.get_logger().warn('Rejecting invalid goal')
            return GoalResponse.REJECT

        self.get_logger().info(f'Accepting goal: order={goal_request.order}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation requests"""
        self.get_logger().info('Cancellation requested')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal with feedback"""
        self.get_logger().info('Executing goal...')

        # Initialize sequence
        sequence = [0, 1]
        order = goal_handle.request.order

        # Compute Fibonacci sequence
        for i in range(1, order):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = sequence
                return result

            # Compute next number
            sequence.append(sequence[i] + sequence[i-1])

            # Send feedback
            feedback = Fibonacci.Feedback()
            feedback.sequence = sequence
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f'Feedback: {sequence}')

            # Simulate computation time
            time.sleep(0.5)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    server = FibonacciActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
```

**Key points:**
- `goal_callback` validates and accepts/rejects goals
- `cancel_callback` handles cancellation requests
- `execute_callback` performs the work and sends feedback
- Check `is_cancel_requested` regularly to support cancellation
- Call `goal_handle.succeed()` or `goal_handle.canceled()` to set final state

### Implementing an Action Client

Here's the corresponding client:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        """Send goal to action server"""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback during execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Success! Result: {result.sequence}')
        elif status == 5:  # CANCELED
            self.get_logger().warn(f'Canceled. Partial: {result.sequence}')

        rclpy.shutdown()
```

## 3. Choosing the Right Pattern

| Pattern | Use Case | Duration | Feedback | Cancellable |
|---------|----------|----------|----------|-------------|
| **Topic** | Streaming data | Continuous | No | N/A |
| **Service** | Quick queries | &lt;1 second | No | No |
| **Action** | Long tasks | Seconds-minutes | Yes | Yes |

### Decision Tree

```
Do you need bidirectional communication?
├─ No → Use Topic
└─ Yes
   ├─ Is it quick (&lt;1 sec)?
   │  └─ Yes → Use Service
   └─ No (long-running)
      └─ Use Action
```

### Example Scenarios

**Scenario 1: Robot Battery Monitor**
- Pattern: **Topic**
- Why: Continuous stream of battery readings
- Publisher: Battery driver node
- Subscribers: Display node, power management node

**Scenario 2: Get Current Joint Positions**
- Pattern: **Service**
- Why: Quick query, immediate response needed
- Server: Joint state controller
- Client: Motion planning node

**Scenario 3: Navigate to Kitchen**
- Pattern: **Action**
- Why: Long-running, needs feedback, might need cancellation
- Server: Navigation stack
- Client: Task planning node

**Scenario 4: Pick and Place Object**
- Pattern: **Action**
- Why: Multi-step task, feedback on progress, cancellable
- Server: Manipulation controller
- Client: High-level task planner

## 4. Best Practices

### Service Best Practices

1. **Keep callbacks fast**: Service callbacks should complete in &lt;100ms
2. **Validate inputs**: Check request parameters before processing
3. **Handle errors gracefully**: Return error codes or status in response
4. **Use meaningful names**: `get_battery_level` not `service1`
5. **Document behavior**: Specify expected inputs, outputs, and edge cases

### Action Best Practices

1. **Send regular feedback**: Update clients every 0.1-1 seconds
2. **Check cancellation**: Call `is_cancel_requested` in loops
3. **Validate goals**: Reject invalid goals early in `goal_callback`
4. **Set final state**: Always call `succeed()`, `canceled()`, or `abort()`
5. **Provide meaningful results**: Include summary statistics or final state

### Common Pitfalls

❌ **Using services for long operations**: Blocks the client
- ✅ Solution: Use actions instead

❌ **Not checking if service exists**: Client crashes if server unavailable
- ✅ Solution: Use `wait_for_service()` with timeout

❌ **Forgetting to check cancellation**: Action can't be stopped
- ✅ Solution: Check `is_cancel_requested` regularly

❌ **Blocking in service callbacks**: Degrades system performance
- ✅ Solution: Keep callbacks fast or use actions

## Practical Example: Robot Task Coordinator

Let's combine services and actions in a realistic scenario:

```python
class RobotTaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')

        # Service client to check if robot is ready
        self.ready_client = self.create_client(
            Trigger, 'check_robot_ready'
        )

        # Action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    async def execute_mission(self):
        # 1. Check if robot is ready (service)
        if not await self.is_robot_ready():
            self.get_logger().error('Robot not ready')
            return

        # 2. Navigate to target (action)
        success = await self.navigate_to_goal(target_pose)

        if success:
            self.get_logger().info('Mission complete!')
        else:
            self.get_logger().error('Navigation failed')

    async def is_robot_ready(self):
        """Quick check using service"""
        request = Trigger.Request()
        future = self.ready_client.call_async(request)
        response = await future
        return response.success

    async def navigate_to_goal(self, pose):
        """Long-running navigation using action"""
        goal = NavigateToPose.Goal()
        goal.pose = pose

        goal_handle = await self.nav_client.send_goal_async(goal)
        result = await goal_handle.get_result_async()

        return result.status == 4  # SUCCEEDED
```

This example demonstrates:
- Using services for quick checks
- Using actions for long-running tasks
- Combining both patterns in a single node
- Async/await for cleaner code structure

## Summary

In this chapter, we learned:

- **Services** provide synchronous request-response for quick operations
- **Actions** provide asynchronous goal-feedback-result for long-running tasks
- Services are blocking and cannot be cancelled
- Actions support feedback and cancellation
- Choosing the right pattern depends on task duration and requirements
- Services and actions complement topics to provide a complete communication toolkit

**Key Takeaways:**
- Use topics for streaming data
- Use services for quick queries (&lt;1 second)
- Use actions for long tasks with feedback
- Always validate inputs and handle errors
- Check for cancellation in action servers

## Review Questions

1. What is the main difference between a service and an action?
2. When would you use a service instead of an action?
3. Why is it important to check `is_cancel_requested` in action servers?
4. What happens if a service client calls a non-existent service?
5. Can a single action server handle multiple goals simultaneously?
6. What are the three parts of an action definition file?
7. Why should service callbacks be fast?
8. How do you reject a goal in an action server?

## Hands-on Exercises

### Exercise 1: Temperature Converter Service
Create a service that converts between Celsius and Fahrenheit:
- Service definition: takes temperature and unit (C/F)
- Returns converted temperature
- Test with various inputs

### Exercise 2: Countdown Action
Create an action that counts down from N to 0:
- Goal: Starting number
- Feedback: Current count
- Result: Final message
- Support cancellation mid-countdown

### Exercise 3: Robot State Manager
Build a node that:
- Provides a service to query robot state (idle, moving, charging)
- Provides an action to change state with validation
- Sends feedback during state transitions

## Further Reading

- [ROS 2 Services Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Understanding Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- Code examples: `/static/code/ros2/chapter02/`

---

**Next Chapter**: [Python Programming with rclpy →](../week-04/ch03-python-rclpy.md)

**Previous Chapter**: [← ROS 2 Nodes and Topics](ch01-nodes-topics.md)

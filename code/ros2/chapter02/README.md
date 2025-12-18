# Chapter 2: ROS 2 Services and Actions - Code Examples

Code examples demonstrating synchronous request-response patterns (services) and long-running tasks (actions).

## Files

### Services
1. **service_example_server.py** - Service server that adds two integers
2. **service_example_client.py** - Service client that requests addition

### Actions
3. **action_example_server.py** - Action server for Fibonacci sequence computation
4. **action_example_client.py** - Action client with feedback monitoring

## Prerequisites

- ROS 2 Humble or later
- Python 3.10+
- `example_interfaces` package (included in ROS 2 desktop)

## Running the Examples

### Example 1: Services (Request-Response)

Services are for short, synchronous operations where the client waits for a response.

**Terminal 1 - Start the service server:**
```bash
cd /path/to/static/code/ros2/chapter02
python3 service_example_server.py
```

**Terminal 2 - Call the service:**
```bash
python3 service_example_client.py 5 7
```

Expected output:
```
Result: 5 + 7 = 12
```

**Verify service exists:**
```bash
ros2 service list           # Should show /add_two_ints
ros2 service type /add_two_ints  # Shows: example_interfaces/srv/AddTwoInts
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 15}"
```

### Example 2: Actions (Long-Running Tasks)

Actions are for tasks that take time and benefit from feedback and cancellation.

**Terminal 1 - Start the action server:**
```bash
cd /path/to/static/code/ros2/chapter02
python3 action_example_server.py
```

**Terminal 2 - Send a goal:**
```bash
python3 action_example_client.py 10
```

Expected output:
```
Feedback: [0, 1]
Feedback: [0, 1, 1]
Feedback: [0, 1, 1, 2]
...
Goal succeeded!
Final sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

**Test cancellation:**
```bash
# Terminal 2 - Start a long computation
python3 action_example_client.py 30

# Terminal 3 - Cancel the goal (within a few seconds)
ros2 action send_goal --feedback /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
# Then press Ctrl+C to cancel
```

**Verify action exists:**
```bash
ros2 action list            # Should show /fibonacci
ros2 action info /fibonacci # Shows action type and clients/servers
```

## Key Concepts

### Services vs Actions vs Topics

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| **Communication** | Many-to-many | One-to-one | One-to-one |
| **Pattern** | Publish/Subscribe | Request/Response | Goal/Feedback/Result |
| **Timing** | Asynchronous | Synchronous | Asynchronous |
| **Feedback** | No | No | Yes |
| **Cancellation** | N/A | No | Yes |
| **Use Case** | Streaming data | Quick queries | Long tasks |

### Services

**When to use:**
- Getting robot state (battery level, joint positions)
- Triggering actions (start/stop recording, reset odometry)
- Configuration queries
- Short computations (<1 second)

**Characteristics:**
- Synchronous: client blocks until response
- No intermediate feedback
- Cannot be cancelled
- Reliable delivery

### Actions

**When to use:**
- Navigation to a goal
- Manipulation tasks
- Trajectory execution
- Long computations
- Tasks that need progress monitoring

**Characteristics:**
- Asynchronous: client doesn't block
- Provides feedback during execution
- Can be cancelled mid-execution
- Three-part communication: goal → feedback → result

## Code Structure

### Service Server Pattern
```python
class MyServiceServer(Node):
    def __init__(self):
        self.srv = self.create_service(
            ServiceType,
            'service_name',
            self.callback
        )

    def callback(self, request, response):
        # Process request
        # Fill response
        return response
```

### Service Client Pattern
```python
class MyServiceClient(Node):
    def __init__(self):
        self.cli = self.create_client(ServiceType, 'service_name')
        self.cli.wait_for_service()

    def send_request(self, data):
        request = ServiceType.Request()
        # Fill request
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

### Action Server Pattern
```python
class MyActionServer(Node):
    def __init__(self):
        self._action_server = ActionServer(
            self,
            ActionType,
            'action_name',
            execute_callback=self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        # Long-running task with feedback
        feedback_msg = ActionType.Feedback()
        goal_handle.publish_feedback(feedback_msg)

        # Check cancellation
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result

        goal_handle.succeed()
        return result
```

### Action Client Pattern
```python
class MyActionClient(Node):
    def __init__(self):
        self._action_client = ActionClient(self, ActionType, 'action_name')

    def send_goal(self, data):
        goal_msg = ActionType.Goal()
        # Fill goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
```

## Exercises

1. **Create a custom service:**
   - Define a service that converts temperature (Celsius ↔ Fahrenheit)
   - Implement server and client

2. **Extend the action server:**
   - Add validation to reject orders > 20
   - Add configurable delay parameter
   - Implement different sequence types (factorial, prime numbers)

3. **Combine patterns:**
   - Create a robot control system with:
     - Service: Get current position
     - Action: Move to target position with progress feedback
     - Topic: Publish position updates

4. **Error handling:**
   - Add timeout handling to service client
   - Implement retry logic for failed service calls
   - Handle action server unavailability gracefully

## Troubleshooting

**Problem:** Service client hangs waiting for server
**Solution:**
- Check server is running: `ros2 service list`
- Verify service name matches exactly
- Check for typos in service type

**Problem:** Action feedback not received
**Solution:**
- Ensure feedback_callback is registered in send_goal_async()
- Check server is publishing feedback with `goal_handle.publish_feedback()`
- Verify message types match

**Problem:** Cannot cancel action
**Solution:**
- Check cancel_callback returns CancelResponse.ACCEPT
- Verify execute_callback checks `goal_handle.is_cancel_requested`
- Ensure goal_handle.canceled() is called before returning

## Further Reading

- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Understanding Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Understanding Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

## License

MIT License - Educational purposes only.

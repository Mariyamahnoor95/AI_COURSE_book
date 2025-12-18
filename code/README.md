# Code Examples

This directory contains executable code examples for the Physical AI & Humanoid Robotics textbook.

## Structure

```
code/
├── foundations/
│   ├── chapter01/
│   └── chapter02/
├── ros2/
│   ├── chapter01/
│   │   ├── hello_publisher.py
│   │   ├── hello_subscriber.py
│   │   └── wall_follower.py
│   ├── chapter02/
│   └── ...
├── digital-twin/
│   ├── chapter07/
│   └── ...
├── isaac/
│   ├── chapter11/
│   └── ...
└── vla/
    ├── chapter17/
    └── ...
```

## Usage

Each chapter directory contains:
- **Standalone scripts**: Can be run directly with `python3 script_name.py`
- **ROS 2 packages**: Use `colcon build` and `ros2 run`
- **README.md**: Setup instructions and dependencies
- **requirements.txt**: Python dependencies (where applicable)

## Prerequisites

- Python 3.10+
- ROS 2 Humble or later
- See individual chapter READMEs for specific requirements

## Running Examples

### ROS 2 Examples

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run a publisher
python3 ros2/chapter01/hello_publisher.py

# In another terminal, run subscriber
python3 ros2/chapter01/hello_subscriber.py
```

### Simulation Examples

```bash
# Launch Gazebo simulation
ros2 launch gazebo_examples world.launch.py

# Run control node
python3 digital-twin/chapter07/robot_controller.py
```

## Contributing

When adding new code examples:

1. Follow PEP 8 style guidelines
2. Include inline comments explaining key concepts
3. Add a README.md with:
   - Purpose and learning objectives
   - Prerequisites
   - Step-by-step running instructions
   - Expected output
4. Test code thoroughly before committing

## License

Code examples are provided for educational purposes under the MIT License.

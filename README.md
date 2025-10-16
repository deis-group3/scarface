# Scarface
ROS2 Jazzy adapter to connect to the all mighty Scarface robot fish

## Overview
This package provides a ROS2 Jazzy interface for controlling and monitoring the Scarface robot fish. It uses modern Python tooling with UV for dependency management and pyproject.toml for configuration.

## Prerequisites
- ROS2 Jazzy installed
- Python 3.10 or higher
- UV (Python package manager)

## Installation

### Clone the Repository
```bash
cd /home/johannes/projects  # or your preferred location
git clone <repository-url> scarface
cd scarface
```

### Install UV (for development tools)
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Build the ROS2 Package
```bash
# Build the workspace (from repository root)
cd ros2_ws
colcon build

# Or build from anywhere
cd /home/johannes/projects/scarface/ros2_ws
colcon build
```

### Setup Environment
```bash
# From repository root
source setup_env.sh

# Or manually
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

## Usage

### Running the node
```bash
# Launch the Scarface node
ros2 launch scarface scarface.launch.py

# Or run the node directly
ros2 run scarface scarface_node
```

### Topics

#### Published Topics
- `/scarface/status` (std_msgs/String): Status updates from the robot fish

#### Subscribed Topics
- `/scarface/command` (std_msgs/String): Commands to control the robot fish

### Testing commands
```bash
# Listen to status messages
ros2 topic echo /scarface/status

# Send a command
ros2 topic pub /scarface/command std_msgs/String "data: 'swim_forward'" --once
```

## Development

### Note on pyproject.toml and setup.py
This package uses both files:
- `setup.py`: Required by ROS2's colcon build system
- `pyproject.toml`: Used by UV for managing development dependencies (pytest, black, ruff, etc.)

The `pyproject.toml` has the `[project]` section commented out to avoid conflicts with `setup.py` during colcon builds.

### Using UV for dependency management
```bash
# Add a new dependency
uv add <package-name>

# Add a development dependency
uv add --dev <package-name>

# Update dependencies
uv sync
```

### Running tests
```bash
# Run pytest tests
uv run pytest

# Run with coverage
uv run pytest --cov=scarface --cov-report=html
```

### Code formatting and linting
```bash
# Format code with black
uv run black scarface/

# Lint code with ruff
uv run ruff check scarface/
```

### ROS2 testing
```bash
# Build and test with colcon
cd /home/johannes/projects/scarface/ros2_ws
colcon build
colcon test
colcon test-result --verbose
```

## Configuration
Edit `config/scarface_params.yaml` to adjust robot parameters:
- `publish_rate`: Status update frequency (Hz)
- `max_speed`: Maximum swimming speed (m/s)
- `max_turn_rate`: Maximum turning rate (rad/s)
- Safety parameters

## Project Structure
```
scarface/                      # Git repository root
├── ros2_ws/                   # ROS2 workspace
│   ├── src/
│   │   └── scarface/          # ROS2 package
│   │       ├── scarface/      # Python package source
│   │       │   ├── __init__.py
│   │       │   └── scarface_node.py
│   │       ├── launch/        # Launch files
│   │       │   └── scarface.launch.py
│   │       ├── config/        # Configuration files
│   │       │   └── scarface_params.yaml
│   │       ├── test/          # Unit tests
│   │       │   └── test_scarface_node.py
│   │       ├── resource/      # ROS2 resource marker
│   │       ├── package.xml    # ROS2 package manifest
│   │       ├── pyproject.toml # Python project config
│   │       └── setup.py       # ROS2 setup file
│   ├── build/                 # Build artifacts (gitignored)
│   ├── install/               # Install artifacts (gitignored)
│   └── log/                   # Build logs (gitignored)
├── .venv/                     # UV virtual environment (gitignored)
├── README.md
├── setup_env.sh               # Convenience script
├── .gitignore
└── .git/
```

**Benefits of this structure:**
- ✅ Everything is git-tracked in one repository
- ✅ Standard ROS2 workspace structure (`ros2_ws/src/`)
- ✅ Easy to clone and build
- ✅ Can add more packages to `ros2_ws/src/` later
- ✅ Build artifacts are separate from source

## License
MIT

## Maintainers
DEIS Group 3

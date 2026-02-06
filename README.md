# Florence-2 ROS2 Wrapper

A robust ROS2 Jazzy wrapper for the Florence-2 baseline and large foundational models using Python 3.12, designed to run in a dedicated virtual environment.

## Tested On
- **OS**: Ubuntu 24.04
- **ROS2 Distribution**: Jazzy
- **Python Version**: 3.12
- **CUDA Version**: 13.0
- **Core Packages**: 
  - `torch==2.5.1`
  - `transformers==4.45.2`

## Installation

### 1. Create and Activate the Virtual Environment
```bash
python3 -m venv ~/virtual-environments/florence2
source ~/virtual-environments/florence2/bin/activate
```

### 2. Install Python Dependencies
Install the package dependencies alongside `colcon-common-extensions` (this is crucial so ROS2 uses the virtual environment's Python parser instead of the system's one when building the node):
```bash
pip install colcon-common-extensions
pip install -r src/florence2_ros2_wrapper/florence2_ros2/requirements.txt
```
*(Ensure `empy` and `lark` are also installed in your environment if you encounter custom message generation issues: `pip install empy==3.3.4 lark`)*

### 3. Install ROS2 System Dependencies
Ensure you have sourced your main ROS2 installation, then use `rosdep` to install required ROS2 packages like `vision_msgs`:
```bash
cd ~/ros2_ws
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
```

### 4. Build the Package
From your workspace root, build the packages:
> **CRITICAL WARNING:** You **MUST** activate your virtual environment before running `colcon build`. If you do not, ROS2 will compile the node using your system's default Python interpreter, which may not have `torch` and `transformers` installed.

```bash
cd ~/ros2_ws
source ~/virtual-environments/florence2/bin/activate
colcon build --packages-select florence2_interfaces florence2_ros2 --symlink-install
```

## Usage

### Running the Node
Always source the virtual environment and your workspace before running the node:
```bash
source ~/virtual-environments/florence2/bin/activate
source ~/ros2_ws/install/setup.bash

# Run using the launch file
ros2 launch florence2_ros2 florence2_launch.py
```

### Parameters
- `model_name` (string): The Florence-2 model variant to load. Default: `microsoft/Florence-2-large-ft`.
- `continuous_task` (string): A task prompt to execute on every incoming frame automatically (e.g., `<OD>`). Leave empty to run in on-demand Service mode.
- `image_topic` (string): Topic providing the input `sensor_msgs/Image`. Default: `/camera/image_raw`.

### Supported Task Modes
Florence-2 supports a variety of Vision-Language tasks. Here are the primary modes you can pass to the `continuous_task` parameter or the `ExecuteTask` service:

- `"<OD>"`: **Object Detection.** Detects common objects and returns bounding boxes with labels.
- `"<CAPTION>"`: **Captioning.** Generates a brief, high-level caption of the image.
- `"<DETAILED_CAPTION>"`: **Detailed Captioning.** Generates a more descriptive and nuanced caption.
- `"<MORE_DETAILED_CAPTION>"`: **More Detailed Captioning.** Generates a highly detailed, comprehensive description of the image.
- `"<DENSE_REGION_CAPTION>"`: **Dense Region Captioning.** Generates distinct captions for various dense regions/objects within the image along with their bounding boxes.
- `"<REGION_PROPOSAL>"`: **Region Proposal.** Finds potential object regions and bounding boxes without explicitly labeling them.
- `"<OCR>"`: **Optical Character Recognition.** Extracts text found in the image.
- `"<OCR_WITH_REGION>"`: **OCR with Regions.** Extracts text and provides the exact bounding boxes where the text is located.

> **Note on formatting:** When passing these parameter values in bash, always wrap them in quotes (e.g., `continuous_task:="<OD>"`), otherwise your terminal might interpret the `<` and `>` characters as input/output redirection operators.

### On-Demand Inference (Service)
You can call the service `~/execute_task` when `continuous_task` is disabled or empty.
```bash
ros2 service call /florence2_node/execute_task florence2_interfaces/srv/ExecuteTask "{task: '<OD>'}"
```

### Subscribed Topics
- `/camera/image_raw` (`sensor_msgs/Image`): The input image stream.

### Published Topics
- `~/annotated_image` (`sensor_msgs/Image`): The input image overlaid with bounding boxes and labels (used for Object Detection).
- `~/detections` (`vision_msgs/Detection2DArray`): Standard ROS2 vision messages for detections.
- `~/results_json` (`std_msgs/String`): The raw JSON output from Florence-2, allowing maximum flexibility for all supported tasks.

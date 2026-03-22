# Florence-2 ROS2 Wrapper

A robust ROS2 Jazzy wrapper for the Florence-2 baseline and large foundational models using Python 3.12. This project is designed to run either via a dedicated Docker container or locally using a Python virtual environment.

## ✨ Features

- **Multiple Vision-Language tasks** — Object Detection, Captioning, OCR, Open Vocabulary Detection, and more, all through a single ROS2 node.
- **Three inference modes** — Continuous (per-frame), on-demand synchronous (Service), and asynchronous with feedback and cancellation (Action).
- **Two Docker profiles** — A lightweight image (~8–10 GB) for hosts with an NVIDIA driver, and a self-contained CUDA image (~12–14 GB) for maximum compatibility.
- **GPU and CPU support** — Automatically uses CUDA when available, falls back to CPU otherwise.

## 📋 Table of Contents

- [Features](#-features)
- [Tested On](#-tested-on)
- [Quick Start](#-quick-start)
- [Running with Docker](#-running-with-docker)
- [Running Locally (Without Docker)](#-running-locally-without-docker)
- [ROS2 Interface](#-ros2-interface)
- [Examples](#-examples)
- [Licence](#-licence)

## 🧪 Tested On

- **OS**: Ubuntu 24.04
- **ROS2 Distribution**: Jazzy
- **Python Version**: 3.12
- **CUDA Version**: 13.0
- **Core Packages**: 
  - `torch==2.5.1`
  - `transformers==4.45.2`

## 🚀 Quick Start

**With Docker (recommended):**
```bash
docker compose up --build
```

**Locally (requires ROS2 Jazzy + virtual environment):**
```bash
source ~/virtual-environments/florence2/bin/activate
source ~/ros2_ws/install/setup.bash
ros2 launch florence2_ros2 florence2_launch.py
```

See the sections below for full installation and configuration details.

## 🐳 Running with Docker

We provide two Docker profiles so you can choose the best trade-off between image size and self-containment. **Both require the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)** installed on your host.

### Option A — Lightweight (Default, recommended)

Uses the official `ros:jazzy-ros-base` image. PyTorch bundles its own CUDA runtime libraries, so no system-level CUDA toolkit is needed inside the container. This produces a **significantly smaller image (~8–10 GB)**.

**Host requirements:** NVIDIA GPU driver (≥ 530) + NVIDIA Container Toolkit.

```bash
docker compose up --build
```

### Option B — Self-contained CUDA

Uses `nvidia/cuda:12.6.3-runtime-ubuntu24.04` as the base. This variant ships CUDA system libraries inside the container for maximum compatibility, at the cost of a larger image (~12–14 GB).

**Host requirements:** NVIDIA GPU driver (≥ 530) + NVIDIA Container Toolkit.

```bash
docker compose -f docker-compose.cuda.yml up --build
```

> [!NOTE]
> **Hugging Face Cache:** Both `docker-compose` files default to mapping your host's `~/.cache/huggingface` to the container so you do not need to re-download the model weights every time the container starts.

### Overriding the Default Command

By default, the container will run `ros2 launch florence2_ros2 florence2_launch.py`.
To run the node with specific parameters (like setting a continuous task or running in service mode), you can modify the `command` field in the corresponding `docker-compose` file, or pass the command inline:

```bash
docker compose run --rm florence2_node ros2 launch florence2_ros2 florence2_launch.py continuous_task:="<OD>" image_topic:=/camera/image_raw
```

## 💻 Running Locally (Without Docker)

If you prefer to install the dependencies locally instead of using Docker, follow these steps:

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
> [!CAUTION]
> You **MUST** activate your virtual environment before running `colcon build`. If you do not, ROS2 will compile the node using your system's default Python interpreter, which may not have `torch` and `transformers` installed.

```bash
cd ~/ros2_ws
source ~/virtual-environments/florence2/bin/activate
colcon build --packages-select florence2_interfaces florence2_ros2 --symlink-install
```

### 5. Launch the Node
You must always source the virtual environment and your workspace before running the node:
```bash
source ~/virtual-environments/florence2/bin/activate
source ~/ros2_ws/install/setup.bash

ros2 launch florence2_ros2 florence2_launch.py
```

## 🤖 ROS2 Interface

The following parameters, topics, and inference modes apply regardless of whether you are running the wrapper via Docker or locally.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `model_name` | string | `microsoft/Florence-2-large-ft` | The Florence-2 model variant to load. |
| `continuous_task` | string | `""` (empty) | A task prompt to execute on every incoming frame automatically (e.g., `<OD>`). Leave empty for on-demand mode. |
| `image_topic` | string | `/camera/image_raw` | Topic providing the input `sensor_msgs/Image`. |

### Supported Task Modes

Florence-2 supports a variety of Vision-Language tasks. You can pass these as the `continuous_task` parameter or to the `ExecuteTask` service/action:

| Task | Prompt | Description |
|---|---|---|
| Object Detection | `<OD>` | Detects common objects and returns bounding boxes with labels. |
| Captioning | `<CAPTION>` | Generates a brief, high-level caption of the image. |
| Detailed Captioning | `<DETAILED_CAPTION>` | Generates a more descriptive and nuanced caption. |
| More Detailed Captioning | `<MORE_DETAILED_CAPTION>` | Generates a highly detailed, comprehensive description. |
| Dense Region Captioning | `<DENSE_REGION_CAPTION>` | Generates distinct captions for various regions along with their bounding boxes. |
| Region Proposal | `<REGION_PROPOSAL>` | Finds potential object regions and bounding boxes without labelling them. |
| OCR | `<OCR>` | Extracts text found in the image. |
| OCR with Regions | `<OCR_WITH_REGION>` | Extracts text and provides the exact bounding boxes where the text is located. |
| Open Vocabulary Detection | `<OPEN_VOCABULARY_DETECTION>` | Detects explicitly requested objects. Requires `text_input` (see example below). |

> [!TIP]
> When passing task prompts in bash, always wrap them in quotes (e.g., `continuous_task:="<OD>"`), otherwise your terminal might interpret the `<` and `>` characters as input/output redirection operators.

### Inference Modes

#### Continuous Mode
Set the `continuous_task` parameter to a task prompt (e.g., `<OD>`) and the node will automatically process every incoming frame on the subscribed image topic.

#### On-Demand Service (Synchronous)
When `continuous_task` is empty, you can call the synchronous service `~/execute_task`. This will block until the model finishes processing.
```bash
ros2 service call /florence2_node/execute_task florence2_interfaces/srv/ExecuteTask "{task: '<OD>'}"
```

For Open Vocabulary Detection, provide the target classes via the `text_input` field:
```bash
ros2 service call /florence2_node/execute_task florence2_interfaces/srv/ExecuteTask "{task: '<OPEN_VOCABULARY_DETECTION>', text_input: 'green chair, laptop'}"
```

#### Asynchronous Action
For complex tasks (like `<MORE_DETAILED_CAPTION>`) or when the robot context changes mid-flight, you can use the Action Server at `~/execute_task_action`. This approach prevents blocking your client and allows you to cancel the inference if needed. It also provides live feedback strings.
```bash
ros2 action send_goal /florence2_node/execute_task_action florence2_interfaces/action/ExecuteTask "{task: '<OD>'}"
```

### Topics

#### Subscribed
| Topic | Type | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | The input image stream. |

#### Published
| Topic | Type | Description |
|---|---|---|
| `~/annotated_image` | `sensor_msgs/Image` | Input image overlaid with bounding boxes and labels (Object Detection). |
| `~/detections` | `vision_msgs/Detection2DArray` | Standard ROS2 vision messages for detections. |
| `~/results_json` | `std_msgs/String` | Raw JSON output from Florence-2, for maximum flexibility across all tasks. |

## 📖 Examples

We have included two example nodes that demonstrate how to interact with the Florence-2 node programmatically.

### Synchronous Service Example (`florence2_service_call_example.py`)

This node subscribes to an image topic, waits until it receives a specific number of frames, and then triggers the `<OD>` service to analyse the last received image, blocking until it finishes.

**To run the example:**
1. In terminal 1, launch the main Florence-2 node (either via Docker or locally in Service mode without the `continuous_task` parameter).
2. In terminal 2, if running locally, source the environment and launch the example node:
```bash
source ~/virtual-environments/florence2/bin/activate
source ~/ros2_ws/install/setup.bash
ros2 launch florence2_ros2 example_launch.py image_topic:=/camera/image_raw
```
3. Play a rosbag or publish images to `/camera/image_raw`. Once the 10th frame is received, the example node will trigger the service call natively and exit.

### Asynchronous Action Example (`florence2_action_example.py`)

This node demonstrates the exact same workflow but utilizing the Action Server. It sends the request asynchronously (`send_goal_async`), processes continuous feedback from the node without blocking, and gracefully shuts down once the final result is returned. 

**To run the action example:**
1. In terminal 1, launch the main Florence-2 node (either via Docker or locally in Service mode without the `continuous_task` parameter).
2. In terminal 2, launch the action example node:
```bash
ros2 run florence2_ros2 florence2_action_example --ros-args -p image_topic:=/camera/image_raw
```

## 📄 Licence

This project is licensed under the MIT Licence. See the [LICENSE](LICENSE) file for details.

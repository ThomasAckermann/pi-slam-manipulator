# Desktop SLAM Arm

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg)](https://docs.ros.org/en/humble/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.8+-red.svg)](https://opencv.org/)

A desktop robotic arm system that uses **Visual SLAM** (Simultaneous Localization and Mapping) to understand and interact with objects in a workspace. Built from scratch with ROS2, OpenCV, and Raspberry Pi for educational purposes.

![Project Demo](docs/images/demo_placeholder.gif)
*Demo: SLAM mapping a desk workspace and tracking objects in real-time*

## 🎯 Project Overview

This project implements a complete robotic perception and manipulation system that:
- **Maps your desk workspace** using a single camera and custom SLAM algorithms
- **Tracks objects** in 3D space with persistent identification
- **Controls a servo-based arm** to interact with detected objects
- **Runs entirely on a Raspberry Pi 4** for compact desktop operation

### Key Features

- 🗺️ **Visual SLAM from scratch** - No black-box libraries, educational implementation
- 📷 **Monocular vision system** - Single Pi Camera for cost-effective setup  
- 🦾 **Servo-controlled arm** - 280° rotation for workspace coverage
- 🎮 **ROS2 architecture** - Industry-standard robotics middleware
- 📊 **Real-time visualization** - RViz2 integration for development and demos
- 🔧 **Modular design** - Each component testable and replaceable

## 🚀 Quick Start

### Prerequisites

- **Raspberry Pi 4** (4GB+ recommended)
- **Pi Camera Module** (v1/v2/HQ)
- **280° Servo Motor** (e.g., MG996R)
- **Ubuntu 22.04** on Pi
- **ROS2 Humble** installed

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/desktop-slam-arm.git
   cd desktop-slam-arm
   ```

2. **Install Python dependencies**
   ```bash
   # Using UV (recommended)
   uv sync
   
   # Or using pip
   pip install -r requirements.txt
   ```

3. **Install ROS2 dependencies**
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Install workspace dependencies
   rosdep install --from-paths src --ignore-src -r -y
   
   # Build the workspace
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Hardware setup**
   ```bash
   # Enable camera interface
   sudo raspi-config  # Interface Options -> Camera -> Enable
   
   # Test camera
   libcamera-hello --preview
   
   # Connect servo to GPIO 18 (see hardware/ for wiring diagram)
   ```

### Running the System

```bash
# Launch the complete system
ros2 launch integration_package slam_arm_system.launch.py

# Or run individual components
ros2 run camera_package camera_node
ros2 run slam_package slam_node  
ros2 run arm_control_package arm_control_node
```

## 📁 Project Structure

```
desktop-slam-arm/
├── src/
│   ├── slam_package/           # Core SLAM implementation
│   │   ├── slam_package/
│   │   │   ├── slam_node.py           # Main SLAM node
│   │   │   ├── feature_detector.py    # ORB feature extraction
│   │   │   ├── pose_estimator.py      # Camera pose estimation
│   │   │   ├── map_builder.py         # 3D map construction
│   │   │   └── loop_closure.py        # Loop closure detection
│   │   └── package.xml
│   ├── camera_package/         # Camera drivers and calibration
│   │   ├── camera_package/
│   │   │   ├── camera_node.py         # Camera interface
│   │   │   ├── calibration.py         # Camera calibration
│   │   │   └── image_processor.py     # Image preprocessing
│   │   └── package.xml
│   ├── arm_control_package/    # Servo control and kinematics
│   │   ├── arm_control_package/
│   │   │   ├── arm_control_node.py    # Servo controller
│   │   │   ├── kinematics.py          # Forward/inverse kinematics
│   │   │   └── trajectory_planner.py  # Motion planning
│   │   └── package.xml
│   └── integration_package/    # System coordination
│       ├── integration_package/
│       │   ├── coordinator.py         # System state management
│       │   └── object_tracker.py      # Object detection & tracking
│       └── package.xml
├── launch/                     # ROS2 launch files
│   ├── slam_arm_system.launch.py     # Complete system
│   ├── slam_only.launch.py           # SLAM development
│   └── camera_calibration.launch.py  # Calibration routine
├── config/                     # Configuration files
│   ├── camera_params.yaml            # Camera calibration
│   ├── slam_params.yaml              # SLAM tuning parameters
│   └── arm_limits.yaml               # Servo constraints
├── hardware/                   # 3D models and wiring
│   ├── arm_mechanism.stl             # 3D printable arm design
│   ├── mounting_bracket.stl          # Camera mount
│   └── wiring_diagram.png            # Hardware connections
├── docs/                       # Documentation
│   ├── SLAM_THEORY.md                # Algorithm explanations
│   ├── HARDWARE_SETUP.md             # Assembly instructions
│   └── API_REFERENCE.md              # Code documentation
├── tests/                      # Test suite
│   ├── unit/                         # Algorithm unit tests
│   ├── integration/                  # Hardware integration tests
│   └── benchmarks/                   # Performance validation
└── tools/                      # Development utilities
    ├── data_collection.py            # Dataset recording
    ├── visualization.py              # Debug visualization
    └── performance_monitor.py        # System profiling
```

## 🔧 Hardware Components

| Component | Model | Purpose | Cost (approx.) |
|-----------|-------|---------|----------------|
| **Compute** | Raspberry Pi 4 (4GB) | Main processing unit | $75 |
| **Camera** | Pi Camera Module v2 | Visual input | $25 |
| **Servo** | MG996R (280°) | Arm actuation | $15 |
| **Power** | 5V 4A USB-C | System power | $15 |
| **Misc** | Jumpers, bracket | Connections, mounting | $10 |
| | | **Total** | **~$140** |

### 3D Printed Parts
- **Arm mechanism** - Single DOF rotating arm
- **Camera mount** - Adjustable angle bracket  
- **Servo bracket** - Desk mounting system

*STL files available in `hardware/` directory*

## 🧠 SLAM Algorithm Overview

Our Visual SLAM implementation follows these key steps:

### 1. **Feature Detection & Matching**
```python
# ORB feature detector for robust keypoints
detector = cv2.ORB_create(nfeatures=1000)
keypoints, descriptors = detector.detectAndCompute(image, None)
```

### 2. **Camera Pose Estimation** 
```python
# Essential matrix estimation + pose recovery
E, mask = cv2.findEssentialMat(pts1, pts2, camera_matrix)
_, R, t, mask = cv2.recoverPose(E, pts1, pts2, camera_matrix)
```

### 3. **3D Point Triangulation**
```python
# Reconstruct 3D points from stereo correspondences
points_4d = cv2.triangulatePoints(P1, P2, pts1, pts2)
points_3d = points_4d[:3] / points_4d[3]  # Normalize
```

### 4. **Bundle Adjustment**
```python
# Optimize camera poses and 3D points jointly
# Using Levenberg-Marquardt for non-linear optimization
optimized_poses, optimized_points = bundle_adjust(poses, points, observations)
```

### 5. **Loop Closure Detection**
```python
# Detect when camera revisits previous locations
# Using bag-of-words with ORB descriptors
if similarity_score > LOOP_CLOSURE_THRESHOLD:
    apply_pose_graph_optimization()
```

## 📊 Performance Metrics

**Target Performance** (Raspberry Pi 4):
- **SLAM Processing**: 5+ FPS
- **Map Accuracy**: 10cm precision for desk objects
- **System Latency**: <200ms camera-to-arm response
- **Memory Usage**: <2GB RAM
- **Power Consumption**: <15W total system

**Current Status**: 
- ✅ Camera pipeline: 15 FPS
- 🔄 SLAM processing: In development
- ⏳ Arm integration: Planned

## 🧪 Testing & Validation

### Unit Tests
```bash
# Run algorithm tests
python -m pytest tests/unit/ -v

# Test specific component
python -m pytest tests/unit/test_feature_detector.py
```

### Integration Tests  
```bash
# Hardware validation
python tests/integration/test_camera_hardware.py
python tests/integration/test_servo_control.py

# System integration
python tests/integration/test_slam_pipeline.py
```

### Performance Benchmarks
```bash
# Profile SLAM performance
python tools/performance_monitor.py --component slam
python tools/performance_monitor.py --component full_system
```

## 🎓 Learning Outcomes

By working through this project, you'll gain hands-on experience with:

**Robotics Fundamentals**
- ROS2 architecture (nodes, topics, services, parameters)
- Coordinate frame transformations and tf2
- Sensor integration and hardware interfacing
- Real-time system constraints and optimization

**Computer Vision & SLAM**
- Camera calibration and intrinsic/extrinsic parameters
- Feature detection and matching algorithms (ORB, SIFT)
- Epipolar geometry and pose estimation
- Bundle adjustment and non-linear optimization
- Loop closure detection and pose graph optimization

**Software Engineering**
- Modular robotics software architecture
- Unit testing for algorithm validation
- Performance profiling and optimization
- Hardware abstraction and driver development

## 🚧 Development Roadmap

### Phase 1: Foundation ✅
- [x] Project setup and repository structure
- [x] ROS2 workspace configuration
- [x] Camera interface and calibration
- [ ] Basic feature detection pipeline

### Phase 2: Core SLAM 🔄
- [ ] Pose estimation from feature matches
- [ ] 3D point triangulation and mapping
- [ ] Basic loop closure detection
- [ ] RViz2 visualization integration

### Phase 3: Object Tracking 📋
- [ ] Object detection integration
- [ ] Persistent object tracking in map coordinates
- [ ] Workspace boundary definition
- [ ] Object interaction planning

### Phase 4: Arm Integration 📋
- [ ] Servo control and safety limits
- [ ] Coordinate transformation (map → arm space)
- [ ] Trajectory planning and execution
- [ ] Demonstration scenarios

### Phase 5: Polish & Optimization 📋
- [ ] Performance tuning for real-time operation
- [ ] Web dashboard for remote monitoring
- [ ] Documentation and tutorials
- [ ] Extended demonstrations

## 🤝 Contributing

Contributions welcome! This is an educational project, so focus on:
- **Clear documentation** of algorithms and design decisions
- **Comprehensive testing** with hardware validation
- **Performance considerations** for Raspberry Pi constraints
- **Educational value** - explain the "why" behind implementations

### Development Setup
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/awesome-improvement`
3. Run tests: `python -m pytest tests/`
4. Commit changes: `git commit -am 'Add awesome improvement'`
5. Push to branch: `git push origin feature/awesome-improvement`
6. Submit a pull request

## 📚 Resources & References

### SLAM Theory
- **"Multiple View Geometry"** - Hartley & Zisserman (essential reference)
- **"Probabilistic Robotics"** - Thrun, Burgard & Fox
- **ORB-SLAM Papers** - Mur-Artal et al. (algorithm inspiration)

### ROS2 Learning
- [Official ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Navigation2 Stack](https://navigation.ros.org/) (advanced reference)

### Computer Vision
- [OpenCV Documentation](https://docs.opencv.org/4.x/)
- [Camera Calibration Guide](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [Feature Matching Tutorial](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html)

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👤 Author

**[Your Name]** - Robotics Engineering Student
- GitHub: [@ThomasAckermann](https://github.com/yourusername)
- LinkedIn: [Thomas Ackermann](https://www.linkedin.com/in/thomas-ackermann-ab38b41a6/)
- Email: uiiiy@student.kit.edu


## 🙏 Acknowledgments

- **ROS2 Community** for the incredible robotics framework
- **OpenCV Contributors** for robust computer vision tools
- **University Robotics Lab** for hardware access and support
- **Open Source SLAM Community** for algorithm inspiration and validation

---

**Project Status**: 🚧 Under Active Development  
**Next Milestone**: Complete Phase 1 foundation components  
**Last Updated**: August 2025

*Built with ❤️ for learning robotics and computer vision*
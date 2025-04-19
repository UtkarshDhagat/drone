# Drone Project Codebase

```markdown
# 🛸 Drone Project Codebase

This repository contains a complete drone vision and simulation system implemented using:
- Python
- OpenCV
- Pangolin (for real-time 3D visualization)
- Custom utilities for feature tracking and trajectory plotting

The system simulates drone movement, visualizes its 3D trajectory, and allows for integration with additional real-time sensor inputs.

---

## 📁 Project Structure

```bash
pango/
├── project/               # Main Python project folder
│   ├── main.py            # Entry point to the visualization and tracking system
│   ├── constants.py       # Camera intrinsics and constants used in the pipeline
│   └── __pycache__/       # Python cache files (ignored)
│
├── Pangolin/              # Submodule for C++ Pangolin visualization library
├── pypangolin/            # Python bindings for Pangolin
├── .gitignore
├── README.md              # This file
```

---

## ⚙️ Features

- Real-time drone trajectory visualization in 3D using Pangolin
- Feature tracking and projection from camera images into 3D space
- Intrinsic matrix and depth modeling for keypoint reconstruction
- Extensible architecture to plug in SLAM/Visual Odometry modules

---

## 🚀 Getting Started

Follow the instructions below to build and run the entire system on macOS/Linux.

### 1. 📦 Clone the Repository

```bash
git clone https://github.com/UtkarshDhagat/drones.git
cd drones
```

> Note: The repository includes `Pangolin` and `pypangolin` directories that might be submodules. If needed, initialize them:
```bash
git submodule update --init --recursive
```

---

### 2. 🛠️ Install Dependencies

Create a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate  # macOS/Linux
```

Install Python packages:

```bash
pip install numpy opencv-python
```

> ⚠️ Make sure `cv2` is installed in the **same environment** where you'll run `main.py`.

---

### 3. 🔧 Build Pangolin and Python Bindings

**Pangolin (C++ Library):**

```bash
cd Pangolin
mkdir build && cd build
cmake ..
make -j4
cd ../..
```

**pypangolin (Python Bindings):**

```bash
cd pypangolin
mkdir build && cd build
cmake ..
make -j4
cd ../..
```

---

### 4. ▶️ Run the Project

Make sure you're in the project root (`pango/`):

```bash
cd project
python main.py
```

This will launch a window using `Pangolin`, showing the live 3D trajectory of a simulated drone (or camera-based system).

---

## 📌 Notes

- `constants.py` defines the intrinsic camera matrix `K` and other utility parameters.
- If you are running on macOS and encounter an error with OpenGL context or window creation, ensure that Pangolin has permission and your display driver supports OpenGL 3+.
- To integrate this with SLAM/VO systems, you can replace the trajectory points and feature tracks accordingly.

---

## 📷 Screenshots

<p align="center">
  <img src="3D%20Drone%20Trajectory.jpeg" alt="3D Drone Trajectory" width="600"/>
</p>


---

## 🧠 Future Enhancements

- Integration with stereo vision or depth camera feeds
- Support for drone control simulation via keyboard/gamepad
- Path planning and obstacle avoidance modules
- ROS2 integration for hardware testing

---

## 🧾 License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## 🙌 Acknowledgements

- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- OpenCV for feature tracking and image handling
- Python community and all open-source contributors

---

## 👨‍💻 Author

**Utkarsh Dhagat**  
Third Year B.Tech, Computer Science  
[Vellore Institute of Technology, Chennai](https://vit.ac.in/)  
📞 +91-9131101011  
🌐 [My Website](https://utkarshdhagat.github.io/Website/)  
🔗 [LinkedIn](https://www.linkedin.com/in/utkarsh-dhagat-3a6a63250/)  
🔗 [GitHub](https://github.com/UtkarshDhagat)

---
```

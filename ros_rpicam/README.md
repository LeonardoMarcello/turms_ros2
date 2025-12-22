**ros_rpicam — Raspberry Pi Camera (libcamera) ROS2 driver**

This package provides a ROS2 wrapper to use the Raspberry Pi camera (Camera Module / CSI cameras) using the libcamera stack and the `rpicam-app` userspace application built from source.

**Overview**
- **What:** A simple ROS2-friendly node that publishes camera frames from the Raspberry Pi camera using `libcamera`/`rpicam-app` as the backend.
- **Why:** Use the modern `libcamera` stack (recommended by Raspberry Pi) instead of legacy drivers. Build `libcamera` and `rpicam-app` from source as described in the Raspberry Pi docs.

**Prerequisites**
- A Raspberry Pi with camera connector and the camera enabled (`raspi-config` → Interface Options → Camera).
- A working Raspberry Pi OS / Debian-based distribution with required build tools.
- The `libcamera` and `rpicam-app` projects built and installed from source as described by Raspberry Pi documentation:
  - Building libcamera and dependencies: https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera
  - `rpicam-app` (or your chosen libcamera app) built from source
- ROS2 (e.g., Humble) workspace environment and `colcon` build tools.

**Recommended system packages**
Run on the Pi before building libcamera (examples):

```bash
sudo apt update && sudo apt install -y git build-essential cmake pkg-config libjpeg-dev libtiff5-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libboost-all-dev python3-pip
sudo pip3 install -U colcon-common-extensions
```

Follow the Raspberry Pi docs for the exact dependencies for `libcamera` and `rpicam-app`.

**Build libcamera & rpicam-app (summary)**
Refer to the official Raspberry Pi instructions linked above. A short summary:

```bash
# Clone libcamera and build (example)
git clone --depth 1 https://git.linuxtv.org/libcamera.git
cd libcamera
meson build
ninja -C build
sudo ninja -C build install

# Clone and build rpicam-app (example)
git clone https://github.com/raspberrypi/rpicam-app.git
cd rpicam-app
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

After the installs, confirm camera works with `libcamera-hello` or `libcamera-jpeg`:

```bash
libcamera-hello -t 2000
libcamera-jpeg -o test.jpg
```

If these test utilities work, the stack is correctly installed.

**ROS2 package: building and running**
- Place this package inside your ROS2 workspace `src/` (e.g., `~/ros2_ws/src/ros_rpicam`).
- Build with `colcon` and source the workspace.

```bash
cd ~/ros2_ws
colcon build --packages-select ros_rpicam
source install/setup.bash
```

**Usage**
- Launch the node (if a launch file is provided) or run the node directly. The node will use the `rpicam-app`/libcamera device and publish images on a topic such as `/camera/image_raw` (check the package-specific node name and topic in your code).

Example (replace with your node name):

```bash
ros2 run ros_rpicam rpicam_node
# or with a launch file
ros2 launch ros_rpicam rpicam_launch.py
```

Subscribe to the image topic with `rqt_image_view` or `ros2 topic echo` (for small message info):

```bash
ros2 run rqt_image_view rqt_image_view
```

**Configuration**
- The node can be configured to change resolution, framerate, or camera options via ROS2 parameters or environment variables depending on the implementation. If the node calls `rpicam-app`, pass options to the app (see `rpicam-app --help`).

**Troubleshooting**
- Camera not found: ensure camera is enabled (`sudo raspi-config`) and system rebooted.
- Permission errors: add your user to the `video` group: `sudo usermod -aG video $USER` then re-login.
- libcamera tests fail: verify you built and installed libcamera and related libraries correctly, and that you followed the Raspberry Pi build instructions closely.
- Device busy / cannot open camera: ensure no other app is holding the camera (e.g., `libcamera-vid`, `vlc`, `rpicam-app` running).
- If frames are dropped or performance is poor: try lowering resolution or framerate, or run the node with higher process priority.

**Extending**
- Add `image_transport` support to publish compressed images.
- Create ROS2 parameters for resolution, framerate, and camera-specific options.
- Add a `launch` file that checks for libcamera availability at startup and prints helpful diagnostics.

**License**
Choose an appropriate license for your code (e.g., MIT, Apache-2.0). Add a `LICENSE` file to the package root.

**References**
- Raspberry Pi Camera software & building libcamera: https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera
- libcamera project: https://www.linuxtv.org/wiki/index.php/Libcamera
- rpicam-app: https://github.com/raspberrypi/rpicam-app

---

If you want, I can:
- Add example `launch` files and a simple node wrapper (if you share your node executable/name).
- Expand the `.gitignore` and CI instructions for cross-building or for GitHub Actions on the Pi.

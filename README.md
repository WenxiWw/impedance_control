# Impedance Controller for Panda Arm

## Functions:
- Simulation of Panda robot using MuJoCo
- Task with State Maschine for playing different Skills/Controllers
- ROS-compatible

## Prerequisites
- Ubuntu 18.04 required (other Ubuntu-versions might be ok as well, but might need a different setup: e.g. Ubuntu 20.04 with ROS Noetic)
- ROS Melodic:
    - Follow installation guide on http://wiki.ros.org/melodic/Installation/Ubuntu
- MuJoCo: https://github.com/deepmind/mujoco/releases
    

- libfranka:
    1. Follow installation guide (from source) on https://frankaemika.github.io/docs/installation_linux.html :
        - You will **not** need to set up the real time kernel for the simulation: This is only needed for controlling the real robot
        - To build libfranka, install the following dependencies from Ubuntu’s package manager:
        ```
        sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
        ```
        - Then, download the source code by cloning libfranka:
        ```
        git clone --recursive https://github.com/frankaemika/libfranka
        cd libfranka
        ```
        - In the source directory, create a build directory and run CMake:
        ```
        mkdir build
        cd build
        cmake -DCMAKE_BUILD_TYPE=Release ..
        cmake --build .
        sudo make install
        ```
    2. configure the dynamic linker run-time bindings:
        ```
        sudo ldconfig
        ```


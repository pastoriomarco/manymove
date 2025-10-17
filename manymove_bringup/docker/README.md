# ManyMove Docker Environments

This directory contains pre-configured Dockerfiles and helper scripts to spin up ROS 2 development containers for ManyMove on either Humble or Jazzy.

## Prerequisites
- Docker Engine installed and running on the host.
- This repository cloned on the host machine.

## Quick start

From the repository root:

```bash
export MANYMOVE_ROS_WS=~/workspaces/dev_ws
${MANYMOVE_ROS_WS}/src/manymove/manymove_bringup/docker/run_manymove_container.sh jazzy
```

Swap the final argument for `humble` to launch the Humble variant. The helper script:
- Builds (or rebuilds) the image automatically when the corresponding Dockerfile changes.
- Clones the `dev` branch of `manymove` inside the image at `/opt/manymove_ws/src/manymove`.
- Runs `rosdep` and `colcon build` during image creation so the overlay is ready out of the box.
- Starts the container without mounting your host workspace, keeping the environment self-contained.
- Forwards X11 variables when available and enables GPU access automatically if `nvidia-smi` exists on the host.
- Forces software OpenGL rendering (via `LIBGL_ALWAYS_SOFTWARE=1`) so RViz works even without direct GPU access; pass your GPU devices and override the variable if you prefer hardware acceleration.

Need custom `docker run` flags? Append `--` and your options:

```bash
./run_manymove_container.sh humble -- -v /data/logs:/logs
```

## Inside the container

The ManyMove overlay is already built and auto-sourced for every interactive shell. Verify with:

```bash
ros2 pkg list | grep manymove
```

Project sources are located at `/opt/manymove_ws/src/manymove`:

```bash
cd /opt/manymove_ws/src/manymove
git status
```

If you pull new changes or switch branches, rebuild with:

```bash
/opt/manymove/setup_workspace.sh
```

Pass additional `colcon build` arguments as needed:

```bash
/opt/manymove/setup_workspace.sh --packages-select manymove_bringup
```

Want hardware-accelerated RViz? Start the container with `--device /dev/dri` (or the equivalent for your GPU) and unset `LIBGL_ALWAYS_SOFTWARE` inside the shell:

```bash
unset LIBGL_ALWAYS_SOFTWARE
rviz2
```

## Manual build (optional)

If you prefer manual control, run from this directory:

```bash
docker build \
  --build-arg ROS_DISTRO=humble \
  --build-arg USERNAME=${USER} \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  --build-arg MANYMOVE_BRANCH=main \
  -t manymove:humble \
  -f Dockerfile.humble \
  .
```

Adjust build arguments (e.g., `MANYMOVE_BRANCH`) and select `Dockerfile.jazzy` for the Jazzy image. The built workspace lives at `/opt/manymove_ws`; mount or override it only if you intentionally want to develop against a host directory.

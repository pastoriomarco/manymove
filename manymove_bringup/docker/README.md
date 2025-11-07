# ManyMove Docker Environments

This directory contains a unified `Dockerfile.manymove` and helper scripts to spin up ROS 2 development containers for ManyMove on either Humble or Jazzy.

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
- Builds (or rebuilds) the image automatically when `Dockerfile.manymove` or the helper scripts change.
- Pass `--pull-latest` to fetch the latest ManyMove commit; the script skips a rebuild if the image already embeds that commit.
- Pass `--build-only` to refresh the image without starting a container (useful before launching composed stacks).
- Clones the ManyMove branch that matches your local checkout (falls back to `main`) inside the image at `/opt/manymove_ws/src/manymove`. Set `MANYMOVE_BRANCH` to override explicitly.
- Runs `rosdep` and `colcon build` during image creation so the overlay is ready out of the box.
- Starts the container without mounting your host workspace, keeping the environment self-contained.
- Forwards X11 variables when available and enables GPU access automatically if `nvidia-smi` exists on the host.
- Forces software OpenGL rendering (via `LIBGL_ALWAYS_SOFTWARE=1`) so RViz works even without direct GPU access; pass your GPU devices and override the variable if you prefer hardware acceleration.

Need custom `docker run` flags? Append `--` and your options:

```bash
export MANYMOVE_ROS_WS=~/workspaces/dev_ws
${MANYMOVE_ROS_WS}/src/manymove/manymove_bringup/docker/run_manymove_container.sh humble -- -v /data/logs:/logs
```

### Refresh ManyMove sources

To update the baked-in ManyMove checkout to the latest commit on the selected branch (and rebuild only if a new commit exists):

```bash
export MANYMOVE_ROS_WS=~/workspaces/dev_ws
${MANYMOVE_ROS_WS}/src/manymove/manymove_bringup/docker/run_manymove_container.sh jazzy --pull-latest
```

The script inspects existing images for the stored commit hash, skips the rebuild if it already matches, and records the resolved commit inside the image at `/opt/manymove_ws/.manymove_commit`. Need to refresh even when nothing changed? Append `--force-rebuild`.

### Inspect baked commits

Use Docker labels to check the commits baked into the images:

```bash
docker image inspect manymove:jazzy --format '{{ index .Config.Labels "manymove.commit" }}'
docker image inspect manymove-xarm:jazzy --format '{{ index .Config.Labels "manymove.xarm.commit" }}'
```

### ManyMove + xArm image

Need the xArm driver baked in? Use the layered helper script:

```bash
export MANYMOVE_ROS_WS=~/workspaces/dev_ws
${MANYMOVE_ROS_WS}/src/manymove/manymove_bringup/docker/run_manymove_xarm_container.sh humble
```

Add `--pull-latest` to refresh both `manymove` and `xarm_ros2` to the newest commits (per ROS distro branch) before rebuilding, `--force-rebuild` to rebuild even when hashes match, or `--build-only` to prime the image without launching it. During the build the resolved xArm commit is written to `/opt/manymove_ws/.xarm_ros2_commit`, and the resulting image tracks its base `manymove:<distro>` source to avoid running stale workspaces.

Hardware acceleration tip: append `-- -e LIBGL_ALWAYS_SOFTWARE=0` when invoking either run script if you want to keep software rendering disabled from the start.

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

### Limit build parallelism

Set `MANYMOVE_COLCON_WORKERS` to a positive integer before launching any of the run scripts (including the color-signal/bootstrap helpers) to cap how many packages `colcon` builds at once inside Docker *and* during `/opt/manymove/setup_workspace.sh`:

```bash
export MANYMOVE_COLCON_WORKERS=2
${MANYMOVE_ROS_WS}/src/manymove/manymove_bringup/docker/run_manymove_container.sh jazzy
```

The same variable applies to `run_manymove_xarm_container.sh` as well as the `manymove_color_signal` overlay. When set, it also drives `CMAKE_BUILD_PARALLEL_LEVEL`, so the Groot build and individual package builds follow the same limit. Unset the variable (or set it to an empty value) to restore the default “auto” worker count.

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
  --build-arg MANYMOVE_COMMIT=$(git ls-remote --heads https://github.com/pastoriomarco/manymove.git main | awk '{print $1}') \
  -t manymove:humble \
  -f Dockerfile.manymove \
  .
```

Adjust build arguments (e.g., `MANYMOVE_BRANCH`) and set `ROS_DISTRO=jazzy` for the Jazzy image. The built workspace lives at `/opt/manymove_ws`; mount or override it only if you intentionally want to develop against a host directory.

For the xArm-enabled image:

```bash
docker build \
  --build-arg ROS_DISTRO=humble \
  --build-arg XARM_BRANCH=humble \
  --build-arg XARM_COMMIT=$(git ls-remote --heads https://github.com/pastoriomarco/xarm_ros2.git humble | awk '{print $1}') \
  -t manymove-xarm:humble \
  -f Dockerfile.manymove_xarm \
  .
```

Set `ROS_DISTRO`/`XARM_BRANCH` to `jazzy` (or your preferred branch) to target Jazzy.

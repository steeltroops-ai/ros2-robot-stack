# --- STAGE 1: SYSTEM SETUP ---
FROM ros:humble-ros-base

USER root
ENV PATH="/root/.bun/bin:${PATH}"
ENV DEBIAN_FRONTEND=noninteractive

# Install core tools and ROS 2 Navigation stack
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    python3-pip \
    unzip \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \

    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-lifecycle-manager \
    python3-colcon-common-extensions \
    && curl -fsSL https://bun.sh/install | bash \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# --- STAGE 2: DEPENDENCY CACHING ---
# Copy ONLY the package configuration to cache the install layer
COPY package.json bun.lock ./
# Copy whatever workspaces exist (resilient to pruning)
COPY apps/ ./apps/
COPY packages/ ./packages/

# Filter out the source code but keep package.jsons and lockfiles for caching
# This maximizes caching while being monorepo friendly
RUN find . -maxdepth 4 -type f -not -name 'package.json' -not -name 'bun.lock' -not -name 'Dockerfile' -delete || true

# Use BUN for ultra-fast multi-workspace installation
# We source ROS setup to ensure rclnodejs builds against the installed ROS libraries
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && bun install"

# --- STAGE 3: BUILD ---
# Now copy the rest of the source
COPY . .

# Build Shared Packages using Bun
WORKDIR /app/packages/shared-types
RUN bun run build

# Prepare Backend (Fix port for HF Spaces)
WORKDIR /app/apps/backend
ENV PORT=7860

# Build ROS 2 Workspaces
WORKDIR /app/robotics/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select simulation_manager amr_navigation"

# --- STAGE 4: RUNTIME ---
WORKDIR /app
COPY hf_entrypoint.sh .
RUN chmod +x hf_entrypoint.sh

# Expose HF Spaces default port
EXPOSE 7860

# Healthcheck to help HF monitor the space
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:7860/api/robots || exit 1

CMD ["./hf_entrypoint.sh"]

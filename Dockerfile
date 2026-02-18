# --- 1. ROS 2 HUMBLE BASE ---
FROM ros:humble-ros-base

# Change to root to install dependencies
USER root

# --- 2. INSTALL SYSTEM DEPENDENCIES ---
# Install Node.js, Bun, and other necessary tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    python3-pip \
    unzip \
    && curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && curl -fsSL https://bun.sh/install | bash \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Add Bun to PATH
ENV PATH="/root/.bun/bin:${PATH}"

# --- 3. WORKSPACE SETUP ---
WORKDIR /app

# Copy the entire monorepo
COPY . .

# --- 4. BUILD SHARED PACKAGES ---
WORKDIR /app/packages/shared-types
RUN npm install && npm run build

# --- 5. BUILD BACKEND ---
WORKDIR /app/apps/backend
RUN npm install
# Fastify needs to listen on port 7860 for Hugging Face Spaces
RUN sed -i 's/port: 4000/port: 7860/g' src/index.ts

# --- 6. BUILD ROS 2 WORKSPACE ---
WORKDIR /app/robotics/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select simulation_manager amr_navigation"

# --- 7. FINAL ENTRYPOINT SCRIPT ---
WORKDIR /app
COPY hf_entrypoint.sh .
RUN chmod +x hf_entrypoint.sh

# Hugging Face Spaces Port
EXPOSE 7860

# Run the entrypoint
CMD ["./hf_entrypoint.sh"]

# Dockerfile
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Base build deps + Eigen + SuiteSparse (for CHOLMOD)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libeigen3-dev \
    libsuitesparse-dev \
    libboost-all-dev \
    qtbase5-dev libqglviewer-dev-qt5 \
    libglew-dev freeglut3-dev \
    libgl1-mesa-dev libglu1-mesa-dev \
 && rm -rf /var/lib/apt/lists/*

# Build and install g2o (shared libs, apps/examples, no OpenGL)
RUN git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git /tmp/g2o && \
    cmake -S /tmp/g2o -B /tmp/g2o/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DG2O_BUILD_APPS=ON \
      -DG2O_BUILD_EXAMPLES=ON \
      -DG2O_USE_OPENGL=OFF && \
    cmake --build /tmp/g2o/build -j"$(nproc)" && \
    cmake --install /tmp/g2o/build && \
    rm -rf /tmp/g2o

RUN mkdir -p /home/ubuntu
COPY ws_ex /home/ubuntu/ws_ex

WORKDIR /home/ubuntu

# syntax=docker/dockerfile:1
FROM ubuntu:24.04 as upstream

# Prevent the interactive wizards from stopping the build
ARG DEBIAN_FRONTEND=noninteractive

# Get the basics
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
  apt-get update -y && apt-get install -q -y --no-install-recommends \
  build-essential        \
  cmake                  \
  gpg                    \
  lsb-release            \
  software-properties-common \
  wget                   \
  && rm -rf /var/lib/apt/lists/*

# Install clang's standard library to use more C++23 features
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
  apt-get update -y && apt-get install -q -y --no-install-recommends \
  libc++abi-dev \
  libc++-dev \
  && rm -rf /var/lib/apt/lists/*

# Install the latest clang to use more C++23 features
RUN wget https://apt.llvm.org/llvm.sh && chmod u+x llvm.sh && ./llvm.sh 18

# Get library dependencies
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
  apt-get update -y && apt-get install -q -y --no-install-recommends \
  libopencv-dev        \
  && rm -rf /var/lib/apt/lists/*

FROM upstream AS development

ARG UID
ARG GID
ARG USER

# fail build if args are missing
# hadolint ignore=SC2028
RUN if [ -z "$UID" ]; then echo '\nERROR: UID not set. Run \n\n \texport UID=$(id -u) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$GID" ]; then echo '\nERROR: GID not set. Run \n\n \texport GID=$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
  apt-get update && apt-get upgrade -y \
  && apt-get install -q -y --no-install-recommends \
  clang-format \
  git \
  inkscape \
  neovim \
  python3 \
  python3-pip \
  python-is-python3 \
  sudo \
  ssh \
  vim \
  wget \
  && rm -rf /var/lib/apt/lists/*

# install developer tools
RUN python3 -m pip install --no-cache-dir --break-system-packages \
  pre-commit

# install hadolint
RUN wget -q -O /bin/hadolint https://github.com/hadolint/hadolint/releases/download/v2.12.0/hadolint-Linux-x86_64 \
  && chmod +x /bin/hadolint

# Setup user home directory
RUN usermod -l $USER ubuntu \
    && groupmod -n $USER ubuntu \
    && usermod -d /home/$USER -m $USER \
    && echo "$USER ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER \
    && touch /home/$USER/.bashrc \
    && chown -R $UID:$GID /home/$USER

USER $USER
ENV SHELL /bin/bash
ENV CC clang-18
ENV CXX clang++-18
ENTRYPOINT []

# Setup mixin
WORKDIR /home/${USER}/ws

FROM ubuntu:24.04



ARG USERNAME=dev
ARG USER_UID=1005
ARG USER_GID=${USER_UID}

ARG LIBFRANKA_VERSION=0.9.2 # change this to your required franka version

ENV DEBIAN_FRONTEND=noninteractive



# system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    sudo \
    pkg-config \
    libeigen3-dev \
    libyaml-cpp-dev \
    capnproto \
    libcapnp-dev \
    libpoco-dev \
    && rm -rf /var/lib/apt/lists/*



# create non root user
RUN EXISTING_USER="$(getent passwd ${USER_UID} | cut -d: -f1)" && \
    if [ -n "${EXISTING_USER}" ]; then echo "UID ${USER_UID} already exists as ${EXISTING_USER}"; \
        if [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
            usermod --login ${USERNAME} "${EXISTING_USER}"; \
            usermod --home /home/${USERNAME} --move-home ${USERNAME}; \
        fi; \
    else \
        EXISTING_GROUP="$(getent group ${USER_GID} | cut -d: -f1)"; \
        if [ -z "${EXISTING_GROUP}" ]; then \
            groupadd --gid ${USER_GID} ${USERNAME}; \
            EXISTING_GROUP=${USERNAME}; \
        fi; \
        useradd -m -s /bin/bash \
            --uid ${USER_UID} \
            --gid "${EXISTING_GROUP}" \
            ${USERNAME}; \
    fi && \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" \
        > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}
# hardware groups
RUN usermod -aG dialout ${USERNAME} || true \
    && usermod -aG video ${USERNAME} || true \
    && usermod -aG plugdev ${USERNAME} || true
    


# install libfranka
WORKDIR /opt
RUN git clone --recurse-submodules --branch ${LIBFRANKA_VERSION} https://github.com/frankarobotics/libfranka.git
WORKDIR /opt/libfranka
RUN cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF
RUN cmake --build build -j$(nproc)
RUN cmake --install build



# create workspace
RUN mkdir -p /workspace \
    && chown -R ${USER_UID}:${USER_GID} /workspace
COPY --chown=${USER_UID}:${USER_GID} . /workspace
USER ${USERNAME}
WORKDIR /workspace




CMD ["/bin/bash"]
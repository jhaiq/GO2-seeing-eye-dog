FROM nvcr.io/nvidia/nemo:26.02

# Install AGIROS loong. The AGIROS deb repo and its GPG key URL follow the
# official image build args (repo.agiros.org.cn, ubuntu2204lts channel).
# NOTE: the nemo base must be Ubuntu 22.04 (jammy) for these debs to resolve;
# otherwise build the loong dev environment from docker/agiros-loong/Dockerfile.
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -fsSL http://repo.agiros.org.cn/debrepo/agiros/ubuntu2204lts/2606/agiros.gpg -o /usr/share/keyrings/agiros.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/agiros.gpg] http://repo.agiros.org.cn/debrepo/agiros/ubuntu2204lts/2606 jammy main" | tee /etc/apt/sources.list.d/agiros.list > /dev/null

RUN apt-get update && apt-get install -y \
    agiros-loong-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up a non-root user matching the host UID/GID (override at build time
# with --build-arg USERNAME=<you> --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g))
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} || true && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} || true && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Source AGIROS loong by default
RUN echo "source /opt/agiros/loong/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]

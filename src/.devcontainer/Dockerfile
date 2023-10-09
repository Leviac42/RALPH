FROM ros:iron
ARG USERNAME=ralph
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip git minicom ros-iron-teleop-twist-joy tmux
RUN git clone https://github.com/Leviac42/RALPH.git /opt/ws_ralph/
RUN chown -R $USERNAME:$USERNAME /opt/ws_ralph/
RUN cd /opt/ws_ralph && pip3 install -r /opt/ws_ralph/requirements.txt && colcon build --symlink-install
RUN usermod -aG sudo ralph && usermod -aG dialout ralph
RUN cp /opt/ws_ralph/ros_entrypoint.sh /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
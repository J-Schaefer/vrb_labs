# Dockerfile of the base image: https://github.com/IntEL4CoRo/docker-stacks/blob/master/Dockerfile
FROM intel4coro/base-notebook:noetic

# Path to the default ROS workspace
ENV ROS_WS=${HOME}/workspace/ros

### Install Dependencies ###

# install giskardpy library
RUN mkdir -p ${HOME}/libs && cd ${HOME}/libs && git clone -b giskard_library https://github.com/SemRoCo/giskardpy.git
RUN cd ${HOME}/libs/giskardpy && git checkout 8b955e3fba81d8878bea06aca6ead6c3cda47af1
RUN cd ${HOME}/libs/giskardpy && pip3 install -r requirements.txt && pip3 install -e .

# Initiate ROS workspace
USER $NB_USER
# Initiate ROS workspace
ENV ROS_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${ROS_WS}/src
# A Workaround for fixing the issue of RvizWeb loading meshes with "file:///" path instead of "package:///".
COPY --chown=${NB_USER}:users binder/me ${ROS_WS}/src/me
#COPY --chown=${NB_USER}:users launch/mujoco_config ${ROS_WS}/src/me/config
COPY --chown=${NB_USER}:users binder/noetic.rosinstall /home/${NB_USER}/noetic.rosinstall
RUN wstool merge /home/${NB_USER}/noetic.rosinstall \
  && wstool update

# Install dependencies
WORKDIR  ${ROS_WS}
USER root
RUN rosdep update --include-eol-distros && apt update && apt dist-upgrade -y \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions \
  && apt install -y ffmpeg libglfw3 libglfw3-dev
WORKDIR ${ROS_WS}
USER ${NB_USER}
RUN pip install -r src/giskardpy/requirements.txt
RUN pip install -r src/me/requirements.txt

# Building Custom bullet3 from source with Python 3.10 (in a clumsy way)
WORKDIR /home/${NB_USER}
RUN git clone https://github.com/pybind/pybind11.git \
  && cd pybind11 \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3 -DPYBIND11_TEST=OFF
USER root
RUN cd pybind11/build && make install
USER ${NB_USER}

RUN cd ${HOME}/libs
RUN git clone https://github.com/SemRoCo/bullet3.git
RUN cd bullet3 && ./build_cmake_pybullet_3.8_double.sh
ENV PYTHONPATH=${PYTHONPATH}:${HOME}/libs/bullet3/build_cmake/better_python:${HOME}/libs/bullet3/examples/pybullet

# Compile the ROS workspace
WORKDIR ${ROS_WS}
RUN catkin build

ENV DISABLE_ROS1_EOL_WARNINGS=1

### Launch JupyterLab ###

# jupyterlab interface configuration
COPY --chown=${NB_USER}:users binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]

FROM ubuntu:22.04


ENV DEBIAN_FRONTEND=noninteractive 

# Update package lists and install necessary tools
RUN apt-get update && apt-get install -y \
    locales \
    python3 \
    python3-pip \
    curl \
    libusb-1.0-0 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8


# Add universe repository and install curl
RUN add-apt-repository universe \
    && apt-get update \
    && apt-get install -y curl

# Set up ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 Humble
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y ros-humble-desktop

# Set Python 3 as the default python
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install Phidget
RUN curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash - && \
    apt-get update && \
    apt-get install -y libphidget22

#install dependancies
RUN pip install Phidget22

# Verify Python installation
RUN python --version
RUN pip3 --version

# Set environment variable
ENV LANG=en_US.UTF-8 

# Set the working directory in the container
WORKDIR /app

# Copy the Temperature.py file from the host to the container
COPY phidget_src .
# Now the structure looks like this '/usr/app/src/phidget_pub.py'

#source ros automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

#run python script automatically
RUN echo "python3 phidget_pub.py" >> ~/.bashrc

#CMD instruction should be used to run the software
#contained by your image, along with any arguments.
CMD ["bash"]
#CMD [ "python3", "phidget_pub.py"]
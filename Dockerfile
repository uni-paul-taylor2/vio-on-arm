# Commands to set this up commented
#docker build -t reproducible-error .
#docker run reproducible-error


# Use an official Ubuntu base image
FROM ubuntu:20.04

# Set environment variables to avoid interactive prompts during package installs
ENV DEBIAN_FRONTEND=noninteractive

# Update and install essential packages
RUN sudo apt update -y
RUN sudo apt upgrade -y
RUN sudo apt install -y gcc g++ build-essential libssl-dev libboost-all-dev
RUN sudo snap install cmake --classic
RUN sudo add-apt-repository -y ppa:git-core/ppa
RUN sudo apt install -y git

# Set working directory and clone repository
WORKDIR /workspace
RUN git clone -b GTSAM-Error https://github.com/uni-paul-taylor2/vio-on-arm.git --depth=1
WORKDIR /workspace/vio-on-arm

# Set the default build command
CMD ["bash", "build.sh"]

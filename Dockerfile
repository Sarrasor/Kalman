FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \ 
	wget \
	build-essential \
    cmake \
    ccache \
    ninja-build \
    clang-tools-14 \
    g++-11 \
    python3 \
    python3-pip \
    git

RUN pip3 install \
	pytest \
    cpplint \
	conan \
    numpy \
	matplotlib

# Takes too long for some reason
# WORKDIR /opt
# RUN git clone https://github.com/Microsoft/vcpkg.git && \
#	 ./vcpkg/bootstrap-vcpkg.sh && vcpkg install opencv

RUN apt-get install -y clang-format-11 clang-tidy-11 cppcheck

RUN apt-get install -y libboost-all-dev


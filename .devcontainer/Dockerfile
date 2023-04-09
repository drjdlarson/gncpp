FROM debian:bullseye-slim

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        openssh-client \
        doxygen \
        graphviz \
        software-properties-common \
        python3-pip \
        python-is-python3 \
        g++ \
        make \
        git \
        curl \
        gpg \
        gpg-agent \
        dirmngr \
    && rm -rf /var/lib/apt/lists/*

# install updated cmake, verify sha file, then verify tar file, then install
RUN cd ~ \
    && curl -OL https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3.tar.gz \
    && curl -OL https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3-SHA-256.txt \
    && curl -OL https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3-SHA-256.txt.asc \
    && gpg --keyserver hkps://keyserver.ubuntu.com --recv-keys C6C265324BBEBDC350B513D02D2CEF1034921684 \
    && gpg --verify cmake-3.26.3-SHA-256.txt.asc cmake-3.26.3-SHA-256.txt \
    && sha256sum -c --ignore-missing cmake-3.26.3-SHA-256.txt \
    && tar -xf cmake-3.26.3.tar.gz \
    && cd cmake-3.26.3 \
    && ./bootstrap -- -DCMAKE_BUILD_TYPE:STRING=Release \
    && make -j4 \
    && make install

# install additional tools for generating test status and coverage badges
RUN pip install genbadge[tests,coverage] gcovr
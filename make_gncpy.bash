#!/bin/bash
conan install . --build=missing -o with_docs=True -o with_tests=True -s build_type=Debug \
    && cmake . --preset conan-debug \
    && cmake --build . --preset conan-debug -j4

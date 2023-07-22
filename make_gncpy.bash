#!/bin/bash
conan install -if build . -o with_tests=True
conan build -if build .
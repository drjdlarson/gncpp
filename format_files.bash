#!/bin/bash
find include/gncpy/ -iname '*.h' -o -iname '*.cpp' | xargs clang-format --style=file -i
find src/ -iname '*.h' -o -iname '*.cpp' | xargs clang-format --style=file -i
find docs/ -iname '*.h' -o -iname '*.cpp' | xargs clang-format --style=file -i
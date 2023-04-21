GNCPY C++
=========
[![Open in Dev Containers](https://img.shields.io/static/v1?label=Dev%20Containers&message=Open&color=blue&logo=visualstudiocode)](https://vscode.dev/redirect?url=vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://github.com/drjdlarson/gncpy_cpp.git) [![Tests Status](https://drjdlarson.github.io/gncpy_cpp/reports/junit/tests-badge.svg?dummy=8484744)](https://drjdlarson.github.io/gncpy_cpp/reports/junit/junit.html) [![Test Cov](https://drjdlarson.github.io/gncpy_cpp/reports/coverage/coverage-badge.svg?dummy=8484744)](https://drjdlarson.github.io/gncpy_cpp/reports/coverage/coverage.html)


A C++ library for Guidance, Navigation, and Control (GNC) algorithms developed by the Laboratory for Autonomy, GNC, and Estimation Research (LAGER) at the University of Alabama (UA). Its primary goal is to serve as the backend for the similarily named python package [GNCPy](https://github.com/drjdlarson/gncpy) however, it can be used as a library in other C++ projects.

This library uses C++20, Doxygen for automatically creating documentation from code comments, and [Google Test](http://google.github.io/googletest/reference/assertions.html) for writing unit tests. CMake is used for generating the Makefiles, and ctest can be used to run the unit tests. This project also follows [semantic versioning](https://semver.org/) where any api breaking changes will increment the major version number. Additionally, a prebuilt docker container image is provided to get started with developing for this library. It contains all of the needed tools to compile the code, run the tests, and build the documentation. The docker container can be used within VS Code through their dev container extension to allow editing local files but compiling using the toolchain provided within the container.


Development Environment Setup
-----------------------------
It is recommended to use VS Code with the dev containers extension for developing. Development containers allow the full toolchain to be automatically setup on most any machine capable of running Docker. For information on dev-containers see [here](https://code.visualstudio.com/docs/devcontainers/containers) for an overview, [here](https://stackoverflow.com/questions/71402603/vs-code-in-docker-container-is-there-a-way-to-automatically-install-extensions) for auto installing extensions in the container
and [here](https://pspdfkit.com/blog/2020/visual-studio-code-cpp-docker/) for an example setup. The provided dev container also has useful extensions installed to ease development.

To being, make sure VS Code and git are installed. Additionally, make sure docker is installed for your system ([Windows](https://docs.docker.com/desktop/install/windows-install/), [Mac](https://docs.docker.com/desktop/install/mac-install/), [Linux](https://docs.docker.com/engine/install/)). Next, install the dev containers extension within VS Code. Clone the repositiory locally on your computer, for windows it is recommended to clone it within your linux subsystem directory (e.g. a sub-directory of your linux home folder) to improve performance within the container (the linux directories on Windows can be accessed through the file browser by typing `\\wsl$` in the address bar and clicking on your distro). Now open the repo folder within VS Code (for windows you may need to connect to the linux subsystem first). Then you should be prompted to open the folder in the container, click yes. If you are not prompted, you can go to the command palette and start typing "Open folder in container". Now your terminal within VS Code will be running commands within the container but the files your are editing/creating will be accessible from your local machine's file browser.

Note if you click the open in container button on the repo's page it will automatically open VS Code, open the container, and clone the repo for you. However, it will do this within a docker volume so the files are only accessible within the container (ie you can't view them through your local file browser).


Example Build
-------------
A typical example build might look something like the following. You edit some files, add some test cases, and want to compile the code and run the tests. You would then run the following (assuming you start at the root of the repository)

```
mkdir build
cd build
cmake -DGNCPY_TEST=ON -DCMAKE_BUILD_TYPE=Debug ..
make -j4
ctest --output-on-failure
```

If the build folder alreay exists you can remove it (or delete its contents). The available library options you can pass to cmake can be show by doing the following

```
cd build
cmake -L .. | grep GNCPY
```

to see all possible cmake options use `cmake -LAH ..` or just see the non-advanced options and their help message run `cmake -LH ..` (both should be run from your build directory).
[![Tests Status](https://ryan4984.github.io/gncpy_cpp/reports/junit/tests-badge.svg?dummy=8484744)](https://ryan4984.github.io/gncpy_cpp/reports/junit/junit.html) [![Test Cov](https://ryan4984.github.io/gncpy_cpp/reports/coverage/coverage-badge.svg?dummy=8484744)](https://ryan4984.github.io/gncpy_cpp/reports/coverage/coverage_nested.html)

Conan testing flow for out-of-source packages
https://bincrafters.github.io/2018/02/27/Updated-Conan-Package-Flow-1.1/

Doxygen must be installed for the documentation to be built. Also recommended to install graphviz for the documentation.

See [here](http://google.github.io/googletest/reference/assertions.html) for reference on gtest (used to write test cases).
You can use ctest to run all the tests (they are built by default when building with cmake).

Development containers can be used in vs code to have the full toolchain automatically setup.
For info on dev-containers see [here](https://code.visualstudio.com/docs/devcontainers/containers) for an overview,
see [here](https://stackoverflow.com/questions/71402603/vs-code-in-docker-container-is-there-a-way-to-automatically-install-extensions) for auto installing extensions in the container
and [here](https://pspdfkit.com/blog/2020/visual-studio-code-cpp-docker/) for an example setup.
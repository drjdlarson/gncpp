Quickstart
==========

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: top

The following subsections give an overview of how to get started developing for the library. This is just one example workflow, others exist but will not be covered here.


Installing the IDE
------------------
The fastest way to get the full toolchain installed is to use the dev containers extension within VS Code. To do this install VS Code and the dev container extension by following their `guide <https://code.visualstudio.com/docs/devcontainers/containers#_getting-started>`_ from "Getting Started" though "Working with Git?". Their guide walks you through installing VS Code, the dev container extension, Docker, and any other needed prereq's based on your operating system.


Extra information on Docker
```````````````````````````
Note, you can think of Docker containers as a tool to allow you to run a mini "Virtual Machine" (VM) and share a folder between the container and your local computer. You have an image which is like the base built version of the container definition (the definition is within a Dockerfile). From this image you can run many containers (i.e. separate instances of the "VM"), with each container being independent. That is, if you install things inside one container, it won't affect what is installed within another container, even if they are from the same image (and installing things within the container does not alter your local PC). This allows the entire toolchain needed to compile the code, run the tests, and generate code coverage reports to be packaged in a reusable and distributatble fashion to ease setup.

Images can be built locally based on the Dockerfile, this capability is provided by the Docker program. Alternatively, prebuilt versions can be downloaded from a central repository such as, Docker hub or Github Package Repository.

The VS Code dev containers extension allows for automatically loading one of these container and adds mechanisms for automatically installing extensions. The definition for what container to use and what other extensions to install are contained in the :code:`.devcontainer/devcontainer.json`. This dev container can also be prebuilt and hosted in a similar fashion to the Docker containers.


Getting the code
----------------
Once the IDE is setup, you can obtain a copy of the code. The recommended way is to clone the repo locally using git. However, you can also click the dev containers button on the documentation or github repo webpages to automatically open the container and clone the repo inside the container. The downside to this approach is the code only exists inside the container, so if the container gets deleted, the code might go with it. Additionally, you won't have access to the files through your local operating systems file browser.

If you clone the repo using git, the next step is to open the top level folder in VS code. You will then be notified that there is a container definition and asked if you want to reopen in the container. Select yes. Once the folder is open in the container, the terminal in VS code will be the terminal in the container. The container is running Debian, so the terminal is a normal Debian terminal (i.e. a bash terminal).


Development
-----------
Most of the development should occur on branches to implement a given feature. Once the feature is implemented, tested and pushed to Github, a Pull Request (PR) can be initiated to bring the code into the master branch. The PR should be merged with the squash strategy, which means all the commits on the branch get combined into one commit when placed on the master branch. This will make it easier to track the addition of features through the commit messages in the releases. An example of the typical flow is

#) Create a branch with a name representing the feature/update to be implemented
#) Get the branch locally (if created on github), and checkout to the branch
#) Make changes to the code, creating commits and pushing to the remote (ie github) as needed
#) Once the feature is implemented and tested, make sure all changes have been committed.
#) Run a :code:`git pull origin master` to make sure there will be no problems with the PR, resolve any merge conflicts and make a commit to close them out (if needed)
#) Start the PR on github
#) Once the PR is approved, merge it to master using the squash startegy and delete the feature branch.
#) On your local branch you can now checkout to master (:code:`git checkout master`), update your local copy (:code:`git pull`), and delete your local copy of the branch that was just merged (CAUTION: make sure the branch was actually merged and the version of master on github contains your changes) (:code:`git branch -D BRANCH_NAME` replace :code:`BRANCH_NAME` with the appropriate value)


Building and testing the code
`````````````````````````````

This code base is setup to use :code:`cmake` for generating the :code:`make` files, and :code:`g++`` for compiling the code. These tools are packaged into the docker container. After editing the code, a build can be started by running the following from the base of the repo

.. code::

    mkdir build
    cd build
    cmake -DGNCPY_TEST=ON -DCMAKE_BUILD_TYPE=Debug ..
    make

This will create an out-of-source build. To reset/cleanup the build, the build folder can be safely deleted and recreated. CMake will go and generate all the necessary Makefiles based on the definitions in the CMakeLists.txt files. Additionaly options to customize the built process can be found by running :code:`cmake -LH ..` from the build directory, all options that start with :code:`GNCPY` are specific to this library. Then :code:`make` can be used to invoke :code:`g++` according to the rules in the Makefiles. The :code:`-j4` flag can be passed to build the code in parallel, where the number represents the number of processors to use in parallel.

The tests utilize Google Test (gtest) to make writing the unit tests easier. They can be run by :code:`ctest` which automatically runs all the tests, collects the output, and can provide summaries of the passing/failing tests. Again, :code:`ctest` has been included in the docker container and gtest gets pulled in and built during the building of the library (if the tests are enabled). After compiling the code, the tests can be run by the following (assuming you are still in the build directory from the out-of-source build process).

.. code:: 

    ctest --output-on-failure

Additional options can be passed to only run tests based upon different criteria. The :code:`-j` flag can als be passed to run tests in parallel (similar to the :code:`make` command).


Creating a Release
``````````````````
After enough features have been implemented, it is time to create a new release (i.e. a tagged snapshot of the repository people can easily download and refer back to). Tools exist within the repo and workflows to automate much of this process. It proceeds as follows

#) Make sure all the code that will be part of the release is on the master branch
#) Checkout to master (:code:`git checkout master`)
#) Update your local copy of master (:code:`git pull`), make sure you have no local changes to master
#) Determine how the version number should be bumped according to the semantic versioning rules (i.e. should "major", "minor", or "patch" version be incremented)
#) Run :code:`python do_release.py TYPE` where :code:`TYPE` is the version type to bump (i.e. should "major", "minor", or "patch")
#) Check that the automated release workflow finishes successfully. This should happen assuming all the prior CI/CD builds where passing and there are no errors in the documentation

The :code:`do_release.py` script increments the appropriate version number in :code:`include/gncpy/core.h`, adds all updated files (this script should be run from a clean master branch so only the :code:`core.h` update should be present) to a commit, pushes the commit to the remote, creates a tag with the new version number, and pushes the tag to the remote. When github sees this version tag it kicks off the release workflow which runs the tests, updates the documentation on the website, and packages a release so it shows up on the releases page.

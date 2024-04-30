import os
import re

from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps


class gncpyRecipe(ConanFile):
    name = "gncpy"
    # version = "0.0.0"
    package_type = "library"

    # Optional metadata
    license = "<Put the package license here>"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of gncpy package here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "with_tests": [True, False],
        "with_docs": [True, False],
        "shared": [
            False,
        ],
        "fPIC": [True, False],
    }
    default_options = {
        "with_tests": False,
        "with_docs": True,
        "shared": False,
        "fPIC": True,
    }

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = (
        "CMakeLists.txt",
        "src/*",
        "include/*",
        "test/*",
        "docs/*",
        "support/*",
        "LICENSE",
        "README.md",
    )

    def layout(self):
        cmake_layout(self)

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def set_version(self):
        with open(
            os.path.join(self.recipe_folder, "include", "gncpy", "core.h")
        ) as fin:
            content = "\n".join(fin.readlines())
            match = re.search("GNCPY_VERSION\s+(\d+)(\d\d)(\d\d)", content)
            if match:
                major = str(int(match.groups()[0]))
                minor = str(int(match.groups()[1]))
                patch = str(int(match.groups()[2]))
                self.version = ".".join([major, minor, patch])
            else:
                raise RuntimeError("Failed to extract version from core.h")

    def validate(self):
        check_min_cppstd(self, "14")
        # check_min_cppstd(self, "20")

    def requirements(self):
        self.requires("boost/1.82.0")
        self.requires("eigen/3.4.0")

        if self.options.with_tests:
            self.requires("gtest/cci.20210126")

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.cache_variables["GNCPY_DOC"] = self.options.with_docs
        tc.cache_variables["GNCPY_TEST"] = self.options.with_tests
        tc.cache_variables["GNCPY_INSTALL"] = True
        tc.cache_variables["GNCPY_LIB_DIR"] = "lib"
        tc.cache_variables["GNCPY_FPIC"] = self.options.fPIC
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["gncpy"]

import os
import re

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout
from conan.tools.build import check_min_cppstd
from conan.tools.scm.git import Git
from conans.tools import load


required_conan_version = ">=1.53.0"


class GncpyConan(ConanFile):
    name = "gncpy"

    # Optional metadata
    license = "<Put the package license here>"
    author = "<Put your name here> <And your email here>"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of Gncpy here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")
    exports_sources = "CMakeLists.txt", "README.rst", "include/*", "src/*", "support/*", "test/*", "example_kf.cpp", "main.cpp"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"with_tests": [True, False], "with_docs": [True, False]}
    default_options = {"with_tests": False, "with_docs": True}

    def configure(self):
        # TODO: this should work but cmake can't find it
        # self.requires.add("cereal/1.3.2")
        if self.options.with_tests:
            self.requires.add("gtest/cci.20210126")

    def set_version(self):
        content = load(os.path.join(self.recipe_folder, "include", "gncpy", "core.h"))
        match = re.search("GNCPY_VERSION\s+(\d+)(\d\d)(\d\d)", content)
        if match:
            major = match.groups()[0]
            minor = str(int(match.groups()[1]))
            patch = str(int(match.groups()[2]))
            self.version = '.'.join([major, minor, patch])
        else:
            raise RuntimeError("Failed to extract version from core.h")

    def layout(self):
        cmake_layout(self)
    
    def validate(self):
        if self.settings.get_safe("compiler.cppstd"):
            check_min_cppstd(self, 20)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["GNCPY_DOC"] = self.options.with_docs
        tc.cache_variables["GNCPY_TEST"] = self.options.with_tests
        tc.cache_variables["GNCPY_INSTALL"] = True
        tc.cache_variables["GNCPY_LIB_DIR"] = "lib"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        target = "gncpy"
        self.cpp_info.set_property("cmake_file_name", "gncpy")
        self.cpp_info.set_property("cmake_target_name", f"lager::{target}")
        self.cpp_info.set_property("pkg_config_name",  "gncpy")

        # TODO: back to global scope in conan v2 once cmake_find_package* generators removed
        postfix = "d" if self.settings.build_type == "Debug" else ""
        libname = "gncpy" + postfix
        self.cpp_info.components["_gncpy"].libs = [libname]
        if self.settings.os == "Linux":
            self.cpp_info.components["_gncpy"].system_libs.extend(["m"])

        # TODO: to remove in conan v2 once cmake_find_package* generators removed
        self.cpp_info.names["cmake_find_package"] = "gncpy"
        self.cpp_info.names["cmake_find_package_multi"] = "gncpy"
        self.cpp_info.names["pkg_config"] = "gncpy"
        self.cpp_info.components["_gncpy"].names["cmake_find_package"] = target
        self.cpp_info.components["_gncpy"].names["cmake_find_package_multi"] = target
        self.cpp_info.components["_gncpy"].set_property("cmake_target_name", f"lager::{target}")

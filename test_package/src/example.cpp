#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <string>

#include "gncpy/core.h"

#define STRINGIFY(x) STRINGIFY2(x)
#define STRINGIFY2(x) #x

int main() {
    std::string version = STRINGIFY(GNCPY_VERSION);
    if (version.length() != 6) {
        std::cerr << "Version string is incorrect length, should be 6 but is "
                  << std::to_string(version.length()) << ". Read as " << version
                  << std::endl;
        return -1;
    }
    std::stringstream ss;
    ss << atoi(version.substr(0, 2).c_str()) << ".";
    ss.clear();
    ss << atoi(version.substr(2, 2).c_str()) << ".";
    ss.clear();
    ss << atoi(version.substr(4, 2).c_str());
    ss.clear();
    std::cout << "lager::gncpy version is " << ss.str() << std::endl;

    return 0;
}

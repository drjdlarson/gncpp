Development Tips and Tricks
===========================

.. contents:: Table of Contents
    :depth: 2
    :local:
    :backlinks: top

The following subsections include some lessons learned and tips to keep in mind when developing for this library. They don't need to be strictly followed but may be helpful when developing or debugging the code.

In general, all development should be done on a branch and squash merged into the master branch after the feature is developed and tests added. After enough features have been added, a release can be created (following semantic versioning for version numbers).

Automated Releases
------------------
When a new release is required, a helper script is available to automate the process. To use this script, make sure all conent for the release has been merged to master and pushed to the remote (i.e. github). Checkout to the master branch on your local PC, make sure you are up to date (i.e. run :code:`git pull`), then run :code:`python do_release.py` from the root of the repository. This script requires you to specify if it is a major, minor, or patch release. Major versions introduce API changes that break backwards compatibility, minor versions introduce new features in a non-API breaking fashion, and patch versions are non-API breaking bug fixes. This script will automatically increment the appropriate version number in **include/gncpy/core.h**, make a commit, create a tag with the version number, and push the commit and tag to the remote (i.e. github). Once the tag gets pushed, the CI/CD pipeline takes over on github to finish the release.



Unit Tests
----------
The unit tests are found within the **test/unit** folder, and subdirectories have been created to mimic the include paths. The top level (**test/unit**) has a CMakeLists.txt file that adds all the subdirectories and any top level tests. Within each subdirectory, there is another CMakeLists.txt that controls how the tests are created for that directory.

When adding a new set of tests, create a new file in the appropriate location, add the test to the CMakeLists.txt file within that directory, and finally, update the CMakeLists.txt file at the root of the repo. The CMakeLists.txt at the root of the repo has special targets for the code coverage that have dependencies on all the unit tests. When you add a new file to the test suite, you need to add this target to the list of dependencies for the code coverage targets at the root of the repo.


Serialization
-------------
The `cereal <https://uscilab.github.io/cereal/>`_ library is used for serialization and some helper macros have be defined in **include/gncpy/SerializeMacros.h** to reduce the boilerplate for adding serialization to a class.

Add Serialization to a class
````````````````````````````
To add serialization to a typical class keep the following in mind.

#) Include the serialization macros header from gncpy
#) Include any archive types from cereal (these can come from the include chain if a parent class already includes them)
#) Include :code:`cereal/access.hpp` and give friend access to the :code:`cereal::access` class
#) If the class is polymorphic (i.e. there is a virtual function or pointers to a base class can also represent child classes) include :code:`cereal/types/polymorphic.hpp`
#) If the class is a child of another serializable class include :code:`cereal/types/base_class.hpp`
#) Some other cereal headers may be required to serialize specific data types (consult their `documentation <https://uscilab.github.io/cereal/>`_ for details)
#) A public construct that takes no arguments is required (this can be defaulted)
#) The :code:`GNCPY_SERIALIZE_CLASS` macro should be used within the class body (but not nested inside any public/private/protected sections) to provide the following methods (this requires :code:`#include <sstream>`in the current header and :code:`#include <cereal/archives/portable_binary.hpp>` in the include chain). If the class can not be instantiated then this macro should not be used and is not needed.

    * :code:`saveClassState`
    * :code:`static loadClass` This can be called without instantiating a class and will return an instance of the class.
    * :code:`toJSON`
    * :code:`createOutputArchive`
    * :code:`static createInputArchive` This needs to be static because it is called when loading the class

#) The class should be directly registered with cereal outside any namespace by passing the class name with all preceeding namespaces to :code:`CEREAL_REGISTER_TYPE`
#) Note that anonymous functions and :code:`std::function` can **not** be serialized.
#) A template specialization for the class should be provided in the private section of the class. :code:`template <class Archive> void serialize(Archive& ar, [[maybe_unused]] const unsigned int version)`. This should use :code:`CEREAL_NVP` around any class values and :code:`cereal::make_nvp` around any base classes to give appropriate names in the JSON form. Additionally, if it is a child class the base class should be passed to the archive using :code:`cereal::virtual_base_class<baseClass_t>(this)` where :code:`baseClass_t` is replaced by the appropriate base class.

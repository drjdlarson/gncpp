#pragma once
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <iostream>
#include <sstream>

// Add save/load, toXML, and create input/output archive functions to a class
// see
// https://www.boost.org/doc/libs/1_80_0/libs/serialization/doc/serialization.html
// for details on save/load class state
#define GNCPY_SERIALIZE_CLASS(Class_t)                                     \
   public:                                                                 \
    std::stringstream saveClassState() const {                             \
        std::stringstream ssb(std::ios_base::binary | std::ios_base::out | \
                              std::ios_base::in);                          \
        {                                                                  \
            boost::archive::binary_oarchive oa(ssb);                       \
            oa << *this;                                                   \
        }                                                                  \
        return ssb;                                                        \
    }                                                                      \
    static Class_t loadClass(std::stringstream& fState) {                  \
        Class_t out;                                                       \
        boost::archive::binary_iarchive ia(fState);                        \
        ia >> out;                                                         \
        return out;                                                        \
    }                                                                      \
    virtual std::string toXML() const {                                    \
        std::stringstream ssb();                                           \
        return "";                                                         \
    }
/*boost::archive::xml_oarchive oa(std::cout);                        \
oa << boost::serialization::make_nvp("test", *this);               \
return "";                                                         \
}*/

#pragma once
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <sstream>

// Add save/load, toJSON, and create input/output archive functions to a class
// see
// https://stackoverflow.com/questions/42253474/trouble-deserializing-cereal-portablebinaryarchive
// for details on save/load class state
#define GNCPY_SERIALIZE_CLASS(Class_t)                    \
   public:                                                \
    std::stringstream saveClassState() const {            \
        std::stringstream ssb(std::ios::binary);          \
        boost::archive::binary_oarchive oa(ssb);          \
        oa << *this;                                      \
        return ssb;                                       \
    }                                                     \
    static Class_t loadClass(std::stringstream& fState) { \
        Class_t out;                                      \
        boost::archive::binary_iarchive ia(fState);       \
        ia >> out;                                        \
        return out;                                       \
    }                                                     \
    virtual std::string toJSON() const {}
/* std::stringstream ss(std::ios::out);
     createOutputArchive<cereal::JSONOutputArchive>(ss);
        return ss.str();
    }   */

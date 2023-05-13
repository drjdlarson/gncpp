#pragma once
#include <cereal/archives/portable_binary.hpp>
#include <sstream>

// Add save/load, toJSON, and create input/output archive functions to a class
// see
// https://stackoverflow.com/questions/42253474/trouble-deserializing-cereal-portablebinaryarchive
// for details on save/load class state
#define GNCPY_SERIALIZE_CLASS(Class_t)                                       \
   public:                                                                   \
    std::stringstream saveClassState() const {                               \
        std::stringstream ssb(std::ios::in | std::ios::out |                 \
                              std::ios::binary);                             \
        createOutputArchive<cereal::PortableBinaryOutputArchive>(ssb);       \
        return ssb;                                                          \
    }                                                                        \
    static Class_t loadClass(std::stringstream& fState) {                    \
        Class_t out;                                                         \
        createInputArchive<cereal::PortableBinaryInputArchive>(fState, out); \
        return out;                                                          \
    }                                                                        \
    virtual std::string toJSON() const {                                     \
        std::stringstream ss(std::ios::out);                                 \
        createOutputArchive<cereal::JSONOutputArchive>(ss);                  \
        return ss.str();                                                     \
    }                                                                        \
                                                                             \
   private:                                                                  \
    template <class Archive>                                                 \
    void createOutputArchive(std::stringstream& os) const {                  \
        Archive ar(os);                                                      \
        ar(*this);                                                           \
    }                                                                        \
    template <class Archive>                                                 \
    static void createInputArchive(std::stringstream& is, Class_t& cls) {    \
        Archive ar(is);                                                      \
        ar(cls);                                                             \
    }

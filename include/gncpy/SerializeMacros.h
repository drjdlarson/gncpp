#pragma once

#define GNCPY_SERIALIZE_TYPES(Class_t) \
    CEREAL_REGISTER_TYPE(Class_t<float>); \
    CEREAL_REGISTER_TYPE(Class_t<double>);

// see https://stackoverflow.com/questions/42253474/trouble-deserializing-cereal-portablebinaryarchive
// for details on save/load class state
#define GNCPY_SERIALIZE_CLASS(Class_t, T) \
    public: \
        std::stringstream saveClassState() { \
            std::stringstream ssb(std::ios::in | std::ios::out | std::ios::binary); \
            createOutputArchive<cereal::PortableBinaryOutputArchive>(ssb); \
            return ssb; \
        } \
        static Class_t<T> loadClass(std::stringstream& fState) { \
            Class_t<T> out; \
            createInputArchive<cereal::PortableBinaryInputArchive>(fState, out); \
            return std::move(out); \
        } \
        virtual std::string toJSON() { \
            std::stringstream ss(std::ios::out); \
            createOutputArchive<cereal::JSONOutputArchive>(ss); \
            return ss.str(); \
        } \
    private: \
        template<class Archive> \
        void createOutputArchive(std::stringstream& os) { \
            Archive ar(os); \
            ar(*this); \
        } \
        template<class Archive> \
        static void createInputArchive(std::stringstream& is, Class_t& cls) { \
            Archive ar(is); \
            ar(cls); \
        }

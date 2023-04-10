#pragma once


#define GNCPY_REGISTER_SERIALIZE_TYPES(Class_t) \
    CEREAL_REGISTER_TYPE(Class_t<float>) \
    CEREAL_REGISTER_TYPE(Class_t<double>)


// see https://stackoverflow.com/questions/42253474/trouble-deserializing-cereal-portablebinaryarchive
// for details on save/load class state
#define GNCPY_SERIALIZE_CLASS(Class_t, T) \
    public: \
        std::stringstream saveClassState() const { \
            std::stringstream ssb(std::ios::in | std::ios::out | std::ios::binary); \
            createOutputArchive<cereal::PortableBinaryOutputArchive>(ssb); \
            return ssb; \
        } \
        static Class_t<T> loadClass(std::stringstream& fState) { \
            Class_t<T> out; \
            createInputArchive<cereal::PortableBinaryInputArchive>(fState, out); \
            return out; \
        } \
        virtual std::string toJSON() const { \
            std::stringstream ss(std::ios::out); \
            createOutputArchive<cereal::JSONOutputArchive>(ss); \
            return ss.str(); \
        } \
    private: \
        template<class Archive> \
        void createOutputArchive(std::stringstream& os) const { \
            Archive ar(os); \
            ar(*this); \
        } \
        template<class Archive> \
        static void createInputArchive(std::stringstream& is, Class_t& cls) { \
            Archive ar(is); \
            ar(cls); \
        }


// see https://uscilab.github.io/cereal/assets/doxygen/polymorphic_8hpp.html#a01ebe0f840ac20c307f64622384e4dae
// and "Registering from a source file" here https://uscilab.github.io/cereal/polymorphism.html
#define GNCPY_POPULATE_SERIALIZE(Class_t, LibName) \
    template class Class_t<float>; \
    template class Class_t<double>; \
    CEREAL_REGISTER_DYNAMIC_INIT(LibName) \
    GNCPY_REGISTER_SERIALIZE_TYPES(Class_t)

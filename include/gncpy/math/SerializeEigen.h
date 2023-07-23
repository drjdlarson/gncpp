#pragma once
#include <Eigen/Dense>
#include <map>
#include <sstream>

namespace boost {
namespace serialization {

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_,
          int MaxCols_>
inline void save(
    Archive& ar,
    const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
    [[maybe_unused]] const unsigned int version) {
    int rows = g.rows();
    int cols = g.cols();

    ar& rows;
    ar& cols;
    ar& boost::serialization::make_array(g.data(), rows * cols);
}

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_,
          int MaxCols_>
inline void load(Archive& ar,
                 Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                 [[maybe_unused]] const unsigned int version) {
    int rows, cols;
    ar& rows;
    ar& cols;
    g.resize(rows, cols);
    ar& boost::serialization::make_array(g.data(), rows * cols);
}

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_,
          int MaxCols_>
inline void serialize(
    Archive& ar, Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
    [[maybe_unused]] const unsigned int version) {
    split_free(ar, g, version);
}

}  // namespace serialization
}  // namespace boost

// namespace cereal {

// // see
// //
// https://stackoverflow.com/questions/22884216/serializing-eigenmatrix-using-cereal-library
// // for details

// // ---------------------------------------------------------
// // For binary archive
// // ---------------------------------------------------------

// template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
//           int _MaxRows, int _MaxCols>
// inline typename std::enable_if<
//     traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value,
//     void>::type
// save(Archive& ar,
//      Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
//      const&
//          m) {
//     int32_t rows = m.rows();
//     int32_t cols = m.cols();
//     ar(rows);
//     ar(cols);
//     ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
// }

// template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
//           int _MaxRows, int _MaxCols>
// inline typename std::enable_if<
//     traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value,
//     void>::type
// load(Archive& ar,
//      Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
//     int32_t rows;
//     int32_t cols;
//     ar(rows);
//     ar(cols);

//     m.resize(rows, cols);

//     ar(binary_data(m.data(),
//                    static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
// }

// // ---------------------------------------------------------
// // For text archive
// // ---------------------------------------------------------

// template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
//           int _MaxRows, int _MaxCols>
// inline
//     typename std::enable_if<traits::is_text_archive<Archive>::value,
//     void>::type save(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols,
//     _Options, _MaxRows,
//                                     _MaxCols> const& m) {
//     int32_t rows = m.rows();
//     int32_t cols = m.cols();
//     ar(cereal::make_nvp("rows", rows));
//     ar(cereal::make_nvp("cols", cols));

//     for (int32_t r = 0; r < rows; r++) {
//         for (int32_t c = 0; c < cols; c++) {
//             std::stringstream name;
//             name << r << "," << c;
//             ar(cereal::make_nvp(name.str(), m(r, c)));
//         }
//     }
// }

// template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
//           int _MaxRows, int _MaxCols>
// inline
//     typename std::enable_if<traits::is_text_archive<Archive>::value,
//     void>::type load(
//         Archive& ar,
//         Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>&
//         m) {
//     int32_t rows = -1;
//     int32_t cols = -1;
//     std::map<std::string, double> data;
//     while (true) {
//         const auto namePtr = ar.getNodeName();

//         if (!namePtr) break;

//         std::string key = namePtr;
//         if (key == "rows") {
//             ar(rows);
//         } else if (key == "cols") {
//             ar(cols);
//         } else {
//             ar(data[key]);
//         }
//     }
//     if (rows >= 0 && cols >= 0) {
//         m.resize(rows, cols);
//         for (auto& [key, val] : data) {
//             size_t pos = key.find(',');
//             int32_t r = std::stoi(key.substr(0, pos));
//             int32_t c = std::stoi(key.substr(pos + 1));
//             m(r, c) = val;
//         }
//     } else {
//         std::runtime_error(
//             "Failed to find rows and columns when deserializing data");
//     }
// }

// }  // namespace cereal
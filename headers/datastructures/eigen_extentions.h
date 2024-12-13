// #ifndef eigen_extensions_H
// #define eigen_extensions_H
// #define ADOLC_TAPELESS

// #include <adolc/adolc_sparse.h>
// #include <adolc/adouble.h>
// #include <Eigen/Core>


// namespace adtl {
//     inline const adouble& conj(const adouble& x)  { return x; }
//     inline const adouble& real(const adouble& x)  { return x; }
//     inline adouble imag(const adouble&)    { return 0.; }
//     inline adouble abs(const adouble&  x)  { return fabs(x); }
//     inline adouble abs2(const adouble& x)  { return x*x; }
// }


// // Taken from the Eigen documentation
// namespace Eigen {

//     typedef Matrix<adouble,Eigen::Dynamic,Eigen::Dynamic> MatrixXad;
//     typedef Vector<adouble,Eigen::Dynamic> VectorXad;

//     template<> struct NumTraits<adouble>
//     : NumTraits<double> // permits to get the epsilon, dummy_precision, lowest, highest functions
//     {
//     typedef adouble Real;
//     typedef adouble NonInteger;
//     typedef adouble Nested;
//     enum {
//         IsComplex = 0,
//         IsInteger = 0,
//         IsSigned = 1,
//         RequireInitialization = 1,
//         ReadCost = 1,
//         AddCost = 3,
//         MulCost = 3
//     };
//     };

//     // // Handle overloading for the adouble matricies
//     // template<typename Scalar, int RowsA, int ColsA, int ColsB>
//     // Eigen::Matrix<adouble,RowsA,ColsB> operator*(const Eigen::Matrix<adouble,RowsA,ColsA>& lhs, const Eigen::Matrix<adouble,ColsA,ColsB>& rhs ){

//     //     Eigen::Matrix<adouble, RowsA, ColsB> result;

//     //     // Manual matrix multiplication
//     //     for (int i = 0; i < RowsA; ++i) {
//     //         for (int j = 0; j < ColsB; ++j) {
//     //             result(i, j) = adouble(0);  // Initialize result with 0
//     //             for (int k = 0; k < ColsA; ++k) {
//     //                 result(i, j) += lhs(i, k) * rhs(k, j);
//     //             }
//     //         }
//     //     }

//     //     return result;

//     // }

//     // template<typename Scalar, int Rows, int Cols>
//     // Eigen::Matrix<adouble,Rows,Cols> operator+(const Eigen::Matrix<adouble,Rows,Cols>& lhs, const Eigen::Matrix<adouble,Cols,Cols>& rhs ){

//     //     Eigen::Matrix<adouble, Rows, Cols> result;

//     //     // Manual matrix multiplication
//     //     for (int i = 0; i < Rows; ++i) {
//     //         for (int j = 0; j < Cols; ++j) {
//     //             result(i, j) = lhs(i,j) + rhs(i,j);
//     //         }
//     //     }

//     //     return result;

//     // }

//     // template<typename Scalar, int Rows, int Cols>
//     // Eigen::Matrix<adouble,Rows,Cols> operator-(const Eigen::Matrix<adouble,Rows,Cols>& lhs, const Eigen::Matrix<adouble,Cols,Cols>& rhs ){

//     //     Eigen::Matrix<adouble, Rows, Cols> result;

//     //     // Manual matrix multiplication
//     //     for (int i = 0; i < Rows; ++i) {
//     //         for (int j = 0; j < Cols; ++j) {
//     //             result(i, j) = lhs(i,j) - rhs(i,j);
//     //         }
//     //     }

//     //     return result;

//     // }

// }

// #endif // ADOLCSUPPORT_H
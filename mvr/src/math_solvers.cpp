#include "math_solvers.h"

#include <boost/numeric/bindings/ublas.hpp>

#define BOOST_NUMERIC_BINDINGS_USE_CLAPACK
#include <boost/numeric/bindings/lapack/driver/gels.hpp>
#undef BOOST_NUMERIC_BINDINGS_USE_CLAPACK 


namespace math_solvers
{
  void least_squares(const ublas::matrix<double>& A, const ublas::vector<double>& b, ublas::vector<double>& x)
  {
    BOOST_UBLAS_CHECK(A.size1() == b.size(), ublas::external_logic());

    ublas::matrix<double> B(b.size(), 1), X;

    ublas::column(B, 0).assign(b);
    least_squares(A, B, X);

    x = ublas::column(X, 0);
  }

  void least_squares(const ublas::matrix<double>& A, const ublas::matrix<double>& B, ublas::matrix<double>& X)
  {
    namespace lapack = boost::numeric::bindings::lapack;

    BOOST_UBLAS_CHECK(A.size1() == B.size1(), ublas::external_logic());

    ublas::matrix<double, ublas::column_major> CA(A), CX((std::max)(A.size1(), A.size2()), B.size2());
    int                                        info;

    ublas::project(CX, ublas::range(0, B.size1()), ublas::range(0, B.size2())).assign(B);

    info = lapack::gels(CA, CX, lapack::optimal_workspace());
    BOOST_UBLAS_CHECK(info == 0, ublas::internal_logic());

    X = ublas::project(CX, ublas::range(0, A.size2()), ublas::range(0, B.size2()));
  }
}
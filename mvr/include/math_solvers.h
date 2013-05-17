# pragma once
#ifndef MATH_SOLVERS_H
#define MATH_SOLVERS_H

#include <boost/numeric/ublas/fwd.hpp>

namespace ublas  = boost::numeric::ublas;

namespace math_solvers {
  void least_squares(const ublas::matrix<double>& A, const ublas::vector<double>& b, ublas::vector<double>& x);
  void least_squares(const ublas::matrix<double>& A, const ublas::matrix<double>& B, ublas::matrix<double>& X);
}


#endif // MATH_SOLVERS_H
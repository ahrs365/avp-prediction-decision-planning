/**
 * @file math.h from https://github.com/sikang/motion_primitive_library
 * @brief Polynomial roots solver
 * Solving real roots for n-th order polynomial:
    if n < 5, the closed form solution will be calculated;
    if n >= 5, using Eigen Polynomials solver which is slower but correct.
 */
#ifndef _CORE_COMMON_INC_COMMON_MATH_POLY_ROOTS_H__
#define _CORE_COMMON_INC_COMMON_MATH_POLY_ROOTS_H_
#include "common/basics/basics.h"
#include <unsupported/Eigen/Polynomials>
#include <iostream>

/// Quadratic equation: \f$b*t^2+c*t+d = 0\f$
inline std::vector<decimal_t> quad(decimal_t b, decimal_t c, decimal_t d) {
  std::vector<decimal_t> dts;
  decimal_t p = c * c - 4 * b * d;
  if (p < 0)
    return dts;
  else {
    dts.push_back((-c - sqrt(p)) / (2 * b));
    dts.push_back((-c + sqrt(p)) / (2 * b));
    return dts;
  }
}

/// Cubic equation: \f$a*t^3+b*t^2+c*t+d = 0\f$
inline std::vector<decimal_t> cubic(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d) {
  std::vector<decimal_t> dts;

  decimal_t a2 = b / a;
  decimal_t a1 = c / a;
  decimal_t a0 = d / a;
  // printf("a: %f, b: %f, c: %f, d: %f\n", a, b, c, d);

  decimal_t Q = (3 * a1 - a2 * a2) / 9;
  decimal_t R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  decimal_t D = Q * Q * Q + R * R;
  // printf("R: %f, Q: %f, D: %f\n", R, Q, D);
  if (D > 0) {
    decimal_t S = std::cbrt(R + sqrt(D));
    decimal_t T = std::cbrt(R - sqrt(D));
    // printf("S: %f, T: %f\n", S, T);
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    decimal_t S = std::cbrt(R);
    dts.push_back(-a2/3+S+S);
    dts.push_back(-a2/3-S);
    return dts;
  }
  else {
    decimal_t theta = acos(R/sqrt(-Q*Q*Q));
    dts.push_back(2*sqrt(-Q)*cos(theta/3)-a2/3);
    dts.push_back(2*sqrt(-Q)*cos((theta+2*M_PI)/3)-a2/3);
    dts.push_back(2*sqrt(-Q)*cos((theta+4*M_PI)/3)-a2/3);
    return dts;
  }
}

/// Quartic equation: \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$
inline std::vector<decimal_t> quartic(decimal_t a, decimal_t b, decimal_t c,
                                      decimal_t d, decimal_t e) {
  std::vector<decimal_t> dts;

  decimal_t a3 = b / a;
  decimal_t a2 = c / a;
  decimal_t a1 = d / a;
  decimal_t a0 = e / a;

  std::vector<decimal_t> ys = cubic(1, -a2, a1*a3-4*a0, 4*a2*a0-a1*a1-a3*a3*a0);
  decimal_t y1 = ys.front();
  //printf("y1: %f\n", y1);
  decimal_t r = a3*a3/4-a2+y1;
  //printf("r: %f\n", r);

  //printf("a = %f, b = %f, c = %f, d = %f, e = %f\n", a, b, c, d, e);
  if(r < 0)
    return dts;

  decimal_t R = sqrt(r);
  decimal_t D, E;
  if(R != 0) {
    D = sqrt(0.75*a3*a3-R*R-2*a2+0.25*(4*a3*a2-8*a1-a3*a3*a3)/R);
    E = sqrt(0.75*a3*a3-R*R-2*a2-0.25*(4*a3*a2-8*a1-a3*a3*a3)/R);
  }
  else {
    D = sqrt(0.75*a3*a3-2*a2+2*sqrt(y1*y1-4*a0));
    E = sqrt(0.75*a3*a3-2*a2-2*sqrt(y1*y1-4*a0));
  }

  if(!std::isnan(D)) {
    dts.push_back(-a3/4+R/2+D/2);
    dts.push_back(-a3/4+R/2-D/2);
  }
  if(!std::isnan(E)) {
    dts.push_back(-a3/4-R/2+E/2);
    dts.push_back(-a3/4-R/2-E/2);
  }

  return dts;
}

/*! \brief General solver for \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$
  \f$a, b, c\f$ can be zero. The function itself checks the highest order of the
  polynomial.
  */
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e) {
  std::vector<decimal_t> ts;
  if (a != 0)
    return quartic(a, b, c, d, e);
  else if (b != 0)
    return cubic(b, c, d, e);
  else if (c != 0)
    return quad(c, d, e);
  else if (d != 0) {
    ts.push_back(-e / d);
    return ts;
  } else
    return ts;
}

/// General solver for \f$a*t^5+b*t^4+c*t^3+d*t^2+e*t+f = 0\f$
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e, decimal_t f) {
  std::vector<decimal_t> ts;
  if (a == 0)
    return solve(b, c, d, e, f);
  else {
    Eigen::VectorXd coeff(6);
    coeff << f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 5> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 5>::RootsType &r = solver.roots();
    std::vector<decimal_t> ts;
    // std::cout << coeff.transpose() << std::endl;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        // std::cout << r[i] << std::endl;
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}

/// General solver for \f$a*t^6+b*t^5+c*t^4+d*t^3+e*t^2+f*t+g = 0\f$
inline std::vector<decimal_t> solve(decimal_t a, decimal_t b, decimal_t c,
                                    decimal_t d, decimal_t e, decimal_t f,
                                    decimal_t g) {
  std::vector<decimal_t> ts;
  if (a == 0 && b == 0)
    return solve(c, d, e, f, g);
  else {
    Eigen::VectorXd coeff(7);
    coeff << g, f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 6> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 6>::RootsType &r = solver.roots();
    std::vector<decimal_t> ts;
    // std::cout << coeff.transpose() << std::endl;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        // std::cout << r[i] << std::endl;
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}

#endif
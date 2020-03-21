/*
 * Copyright (c) 2015, Jonathan Ventura
 * Modified by fw, 2018.10
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * 这个源码连同PolynomialInternal.hpp一起可以独立成为一个工具模块
 * 可以提供多项式的 加、减、乘、带入求值、求根 五种运算
 * 具体使用方法可以参照 http://github.com/hannbusann/Polynomial_math/test 中的例程
 * 
 */
#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP

#include <Eigen/Core>
#include <unsupported/Eigen/Polynomials>
#include <vector>

#include "PolynomialInternal.hpp"

using namespace dmotion;
namespace dmotion
{
/**
     * Polynomial
     *
     * Templated class for representing a polynomial expression.
     *
     * The template parameter indicates the degree of polynomial.  An n-th degree polynomial has n+1 coefficients.
     *
     * The coefficients are stored in this order:
     * c0 x^deg + c1 x^(deg-1) + ...
     *
     * A dynamically-sized version is available: Polynomial<Eigen::Dynamic>
     *
     * Note that the static and dynamic versions should not be mixed, i.e. do not add a dynamic polynomial to a static one.
     */

template <int deg> // deg equals to degree of your polynomial
class Polynomial
{
    Eigen::Matrix<double, deg + 1, 1> coef;
    friend class ThreeInterpolation ;
  public:
    Polynomial() //创建deg阶的系数都为0的多项式
        : coef(Eigen::Matrix<double, 1, deg + 1>::Zero())
    {
    }

    explicit Polynomial(const Eigen::Matrix<double, deg + 1, 1> &coefin)
        : coef(coefin)
    {
    }

    explicit Polynomial(const Polynomial<deg> &polyin)
        : coef(polyin.coef)
    {
    }

    explicit Polynomial(const double *coefin)
        : coef(Internal::vecmap<deg + 1>(coefin))
    {
    }

    const Eigen::Matrix<double, deg + 1, 1> &coefficients() const
    {
        return coef;
    }

    Eigen::Matrix<double, deg + 1, 1> &coefficients()
    {
        return coef;
    }

    template <int degin>
    Polynomial<Internal::max<degin, deg>::value> operator+(const Polynomial<degin> &poly) const
    {
        Polynomial<Internal::max<degin, deg>::value> p;
        p.coefficients().tail(degin + 1) = poly.coefficients();
        p.coefficients().tail(deg + 1) += coef;
        return p;
    }

    template <int degin>
    Polynomial<Internal::max<degin, deg>::value> operator-(const Polynomial<degin> &poly) const
    {
        Polynomial<Internal::max<degin, deg>::value> p;
        p.coefficients().tail(deg + 1) = coef;
        p.coefficients().tail(degin + 1) -= poly.coefficients();
        return p;
    }

    template <int degin>
    Polynomial<degin + deg> operator*(const Polynomial<degin> &poly) const
    {
        //std::cout << "Degree elevation happens!" << std::endl;
        Polynomial<degin + deg> p;
        Internal::PolyConv<deg, degin>::compute(p.coefficients(), coef, poly.coefficients());
        return p;
    }

    Polynomial<deg> operator*(const double c) const
    {
        //std::cout << "Degree still!" << std::endl;
        return Polynomial<deg>(coef * c);
    }

    template <int degin>
    bool operator==(const Polynomial<degin> &poly) const
    {
        return degin == deg;
    }

    template <int degin>
    Polynomial<degin> operator=(const Polynomial<degin> &poly) const
    {
        if (coef == poly.coef)
            return *this;
        Eigen::Matrix<double, degin + 1, 1> coefre; 
        return Polynomial<degin>(coefre);
    }

    template <int degin>
    int getDegree(const Polynomial<degin> &poly) const
    {
        return degin;
    }

    double eval(double x) const
    {
        return Internal::PolyVal<deg>::compute(coef, x);
    }

    void findRoots(std::vector<double> &roots) const
    {
        double lb, ub;
        rootBounds(lb, ub);
        if (coef[0] == 0)
        {
            Internal::SturmRootFinder<deg - 1> sturm(coef.tail(deg));
            sturm.realRoots(lb, ub, roots);
        }
        else
        {
            Internal::SturmRootFinder<deg> sturm(coef);
            sturm.realRoots(lb, ub, roots);
        }
    }

    void rootBounds(double &lb, double &ub) const
    {
        Eigen::Matrix<double, deg, 1> mycoef = coef.tail(deg).array().abs();
        mycoef /= fabs(coef(0));
        mycoef(0) += 1.;
        ub = mycoef.maxCoeff();
        lb = -ub;
    }
};

} // namespace dmotion

#endif

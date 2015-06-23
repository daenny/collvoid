/* This file is adapted from octomap/math/vector3.h see original copyright information below
 */

/*
 * Copyright (c) 2009-2011, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COLLVOID_VECTOR2_H
#define COLLVOID_VECTOR2_H

#include <iostream>
#include <cmath>

namespace collvoid {

    class Vector2
    {
    public:

        Vector2 () { data[0] = data[1] = 0; }


        Vector2 (const Vector2& other) {
            data[0] = other(0);
            data[1] = other(1);
        }

        Vector2 (double x, double y) {
            data[0] = x;
            data[1] = y;
        }

        inline Vector2& operator= (const Vector2& other)  {
            data[0] = other(0);
            data[1] = other(1);
            return *this;
        }

        inline const double& operator() (unsigned int i) const
        {
            return data[i];
        }

        inline double& operator() (unsigned int i)
        {
            return data[i];
        }

        inline double& x()
        {
            return operator()(0);
        }

        inline double& y()
        {
            return operator()(1);
        }

        inline const double& x() const
        {
            return operator()(0);
        }

        inline const double& y() const
        {
            return operator()(1);
        }

        inline Vector2 operator- () const
        {
            Vector2 result;
            result(0) = -data[0];
            result(1) = -data[1];
            return result;
        }

        inline Vector2 operator+ (const Vector2 &other) const
        {
            Vector2 result(*this);
            result(0) += other(0);
            result(1) += other(1);
            return result;
        }

        inline double operator* (const Vector2& other) const
        {
            return x()*other.x() + y()*other.y();
        }

        inline Vector2 operator*  (double x) const {
            Vector2 result(*this);
            result(0) *= x;
            result(1) *= x;
            return result;
        }

        inline Vector2 operator/  (double x) const {
            Vector2 result(*this);
            result(0) /= x;
            result(1) /= x;
            return result;
        }

        inline Vector2 operator- (const Vector2 &other) const
        {
            Vector2 result(*this);
            result(0) -= other(0);
            result(1) -= other(1);
            return result;
        }

        inline void operator+= (const Vector2 &other)
        {
            data[0] += other(0);
            data[1] += other(1);
        }

        inline void operator-= (const Vector2& other) {
            data[0] -= other(0);
            data[1] -= other(1);
        }

        inline void operator/= (double x) {
            data[0] /= x;
            data[1] /= x;
        }

        inline void operator*= (double x) {
            data[0] *= x;
            data[1] *= x;
        }

        inline bool operator== (const Vector2 &other) const {
            for (unsigned int i=0; i<2; i++) {
                if (operator()(i) != other(i))
                    return false;
            }
            return true;
        }

        inline bool operator!=(const Vector2& vector) const
        {
            return x() != vector.x() || y() != vector.y();
        }



        inline double dist (const Vector2& other) const {
            double dist_x = x() - other.x();
            double dist_y = y() - other.y();
            return sqrt(dist_x*dist_x + dist_y*dist_y);
        }


        inline std::ostream& operator<<(std::ostream& os) {
            os << "(" << x() << "," << y() << ")";

            return os;
        }

    protected:
        double data[2];

    };

    inline Vector2 operator*(float s, const Vector2& vector)  {
        return Vector2(s * vector.x(), s * vector.y());
    }



    inline double abs(const Vector2& vector) {
        return std::sqrt(vector * vector);
    }


    inline double absSqr(const Vector2& vector) {
        return vector * vector;
    }

    inline double atan(const Vector2& vector){
        return std::atan2(vector.y(), vector.x());
    }

    inline double det(const Vector2& vector1, const Vector2& vector2) {
        return vector1.x() * vector2.y() - vector1.y() * vector2.x();
    }

    inline Vector2 normalize(const Vector2& vector){
        return vector / abs(vector);
    }

    inline Vector2 normal(const Vector2& vector){
        return normalize(Vector2(vector.y(), -(vector.x())));
    }

    inline double angleBetween(const Vector2& one, const Vector2& two) {
        double dot_prod = one*two;
        double len1 = abs(one);
        double len2 = abs(two);
        return acos(dot_prod / (len1*len2));
    }

    inline Vector2 rotateVectorByAngle(double x, double y, double ang){
        double cos_a, sin_a;
        cos_a = cos(ang);
        sin_a = sin(ang);
        return Vector2(cos_a * x - sin_a * y, cos_a * y + sin_a * x);

    }
    inline Vector2 rotateVectorByAngle(const Vector2& vec, double ang) {
        return rotateVectorByAngle(vec.x(), vec.y(), ang);
    }

}


#endif

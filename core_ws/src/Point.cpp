/**
 * @file Point.cpp
 * @brief defines generic class Point
 * @author Julie Digne
 * @date 2012/10/10
 * @copyright This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @author Mau Hing Yip mauhingyip@hotmail.com
 * @date 2025-06-12
 * @note This file was further developed based on prior work by Julie Digne.
 */

#include "Point.h"
#include <iostream>

Point::Point() : m_x(0.0), m_y(0.0), m_z(0.0)
{
}

Point::Point(double x, double y, double z) : m_x(x), m_y(y), m_z(z)
{
}

Point& Point::operator=(const Point& other)
{
    if (this != &other)
    {
        m_x = other.m_x;
        m_y = other.m_y;
        m_z = other.m_z;
    }
    return *this;
}

Point::Point(const Point& other) : m_x(other.m_x), m_y(other.m_y), m_z(other.m_z)
{
}

Point::~Point()
{
}

double Point::x() const
{
    return m_x;
}

double Point::y() const
{
    return m_y;
}

double Point::z() const
{
    return m_z;
}

std::ostream &operator<<(std::ostream &out, const Point &v)
{
    out << v.x() << "\t" << v.y() << "\t" << v.z();
    return out;
}
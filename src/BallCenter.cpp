#include "BallCenter.h"

BallCenter::BallCenter(double x, double y, double z, Facet* facet) : Point(x, y, z), m_facet(facet) {}

BallCenter::~BallCenter() {
    m_facet = nullptr;
}

BallCenter::BallCenter(const BallCenter& other) : Point(other), m_facet(other.m_facet) {}

ostream& operator << (ostream& out, const BallCenter& v) {
    out << "BallCenter: " << v.x() << " " << v.y() << " " << v.z() << std::endl;
    return out;
}
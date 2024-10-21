#include "BallCenter.h"

BallCenter::BallCenter(double x, double y, double z, Facet* facet) : Point(x, y, z), m_facet(facet) {}

BallCenter::~BallCenter() {
    m_facet = nullptr;
}
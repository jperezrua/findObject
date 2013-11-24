////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"

Transformation::Transformation()
: m_rotation(Matx33f::eye())
, m_translation(Vec3f(0,0,0))
{
  
}

Transformation::Transformation(const Matx33f& r, const Vec3f& t)
: m_rotation(r)
, m_translation(t)
{
  
}

Matx33f& Transformation::r()
{
  return m_rotation;
}

Vec3f&  Transformation::t()
{
  return  m_translation;
}

const Matx33f& Transformation::r() const
{
  return m_rotation;
}

const Vec3f&  Transformation::t() const
{
  return  m_translation;
}

Matx44f Transformation::getMat44() const
{
  Matx44f res = Matx44f::eye();

  for (int col=0;col<3;col++)
  {
    for (int row=0;row<3;row++)
    {
      // Copy rotation component
      res(row,col) = m_rotation(row,col);
    }
    
    // Copy translation component
    res(3,col) = m_translation[col];
  }
  
  return res;
}

Transformation Transformation::getInverted() const
{
    return Transformation(m_rotation.t(), -m_translation);
}

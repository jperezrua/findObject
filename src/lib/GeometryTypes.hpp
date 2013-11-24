#ifndef AR_GeometryTypes_hpp
#define AR_GeometryTypes_hpp

#include <opencv2/opencv.hpp>

using namespace cv;

struct Transformation
{
  Transformation();
  Transformation(const Matx33f& r, const Vec3f& t);
  
  Matx33f& r();
  Vec3f&  t();
  
  const Matx33f& r() const;
  const Vec3f&  t() const;
  
  Matx44f getMat44() const;
  
  Transformation getInverted() const;
private:
  Matx33f m_rotation;
  Vec3f  m_translation;
};

#endif

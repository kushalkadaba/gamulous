#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    // ======
	// TODO:
	// ======
	  t_ = t;
	  r_ = r;
  }

  explicit RigTForm(const Cvec3& t) {
    // ======
	// TODO:
	// ======
	  t_ = t;
	  r_ = Quat();
  }

  explicit RigTForm(const Quat& r) {
    // ======
	// TODO:
	// ======
	  t_ = Cvec3();
	  r_ = r;
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    // ======
	// TODO:
	// ======
	 
	  return r_*a + Cvec4(t_);
  }

  RigTForm operator * (const RigTForm& a) const {
    // ======
	// TODO:
	// ======
	  Quat rotation = r_ * a.r_;
	  Cvec3 translation = Cvec3(Cvec4(t_)+r_*Cvec4(a.t_));
	  return RigTForm(translation,rotation);
  }
};

inline RigTForm inv(const RigTForm& tform) {
    // ======
	// TODO:
	// ======
	Quat rotation = inv(tform.getRotation());
	Cvec3 translation = Cvec3(-(inv(tform.getRotation())*Cvec4(tform.getTranslation())));
	return RigTForm(translation,rotation);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
    // ======
	// TODO:
	// ======
	Matrix4 T = Matrix4::makeTranslation(tform.getTranslation());
	Matrix4 R = quatToMatrix(tform.getRotation());
    return T * R;
}

inline RigTForm const doMtoOwrtA(const RigTForm& M,const RigTForm& O, const RigTForm& A){
	return A*M*inv(A)*O;
}

inline const RigTForm makeMixedFrame(const RigTForm& T,const RigTForm& R){
	return transFact(T)*linFact(R);
}
/*=================================================================
11/08 Linear Interpolation
=================================================================*/
inline RigTForm lerp(const RigTForm& tform0, const RigTForm& tform1, const double alpha)
{
	Cvec3 t = lerp<double,3>(tform0.getTranslation(),tform1.getTranslation(),alpha);
	Quat q = slerp(tform0.getRotation(), tform1.getRotation(), alpha);
	return RigTForm(t,q);
}

inline std::ostream& operator<<(std::ostream& os, RigTForm& r){
	os << r.getRotation()<< std::endl;
	os << r.getTranslation() << std::endl;
	return os;
}
inline std::istream& operator>>(std::istream& is, RigTForm& r){
	double w,x,y,z;
	is >> x >>y>>z;
	r.setRotation(Quat(1,x,y,z));
	is >> x >> y>> z;
	r.setTranslation(Cvec3(x,y,z));
	return is;
}
#endif

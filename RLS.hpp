#include "FreeRTOS.h"
#include "matrix.h"
#include "task.h"

#pragma once

template <uint32_t dim>
class RLS {
 public:
  RLS() = delete;  // 必须只能带参数的构造函数

  constexpr RLS(float delta_,
                float lambda_)
      : dimension_(dim),
        lambda_(lambda_),
        delta_(delta_),
        defaultparamsvector_(matrixf::zeros<dim, 1>()) {
    this->Reset();     // 初始化各个矩阵
    this->Validate();  // 验证参数合法性
  }

  void Reset() {
    transmatrix_ = matrixf::eye<dim, dim>() * delta_;
    gainvector_ = matrixf::zeros<dim, 1>();
    paramsvector_ = matrixf::zeros<dim, 1>();
  }

  const Matrixf<dim, 1> &Update(Matrixf<dim, 1> &sampleVector,
                                float actualOutput) {
    gainvector_ =
        (transmatrix_ * sampleVector) /
        (1.0f +
         (sampleVector.Trans() * transmatrix_ * sampleVector)[0][0] / lambda_) /
        lambda_;
    paramsvector_ +=
        gainvector_ *
        (actualOutput -
         (sampleVector.Trans() * paramsvector_)[0][0]);
    transmatrix_ =
        (transmatrix_ - gainvector_ * sampleVector.Trans() * transmatrix_) /
        lambda_;

    return paramsvector_;
  }

  void SetParamVector(const Matrixf<dim, 1> &updatedParams) {
    paramsvector_ = updatedParams;
    defaultparamsvector_ = updatedParams;
  }

 private:
  void Validate() const {
    configASSERT(lambda_ >= 0.0f || lambda_ <= 1.0f);
    configASSERT(delta_ > 0);
  }

  uint32_t dimension_;
  float lambda_;
  float delta_;

  Matrixf<dim, dim> transmatrix_;
  Matrixf<dim, 1> gainvector_;
  Matrixf<dim, 1> paramsvector_;
  Matrixf<dim, 1> defaultparamsvector_;
};

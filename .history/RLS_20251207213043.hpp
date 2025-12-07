#include "FreeRTOS.h"
#include "matrix.h"
#include "task.h"

#pragma once

template <uint32_t dim>  // Dim 维度
class RLS {
 public:
  /**
   * @brief Delete the default constructor
   */
  RLS() = delete;  // 必须只能带参数的构造函数

  /**
   * @brief The constructor
   * @param delta_ The intialized non-singular value of the transfer matrix
   * @param lambda_ The forgotten index
   */
  constexpr RLS(float delta_,
                float lambda_)  // delat_ 初始化转移矩阵的非零值 lambada_
                                // 遗忘因子 越接近1约记忆历史数据
      : dimension_(dim),
        lambda_(lambda_),
        delta_(delta_),
        lastupdate_(0),
        updatecnt_(0),
        defaultparamsvector_(matrixf::zeros<dim, 1>()) {
    this->Reset();     // 初始化各个矩阵
    this->Validate();  // 验证参数合法性
  }

  constexpr RLS(float delta_, float lambda_, Matrixf<dim, 1> initparam)
      : RLS(delta_, lambda_) {
    defaultparamsvector_ = initparam;
  }

  /**
   * @brief Reset the RLS module
   * @retval None
   */
  void Reset() {
    transmatrix_ = matrixf::eye<dim, dim>() * delta_;
    gainvector_ = matrixf::zeros<dim, 1>();
    paramsvector_ = matrixf::zeros<dim, 1>();
  }

  /**
   * @brief Proccess a cycle of RLS update
   * @param sampleVector The new samples input expressed in n x 1 dimensionasl
   * vector form
   * @param actualOutput The actual feedback real output
   * @retval paramsVector
   */
  // 输入样本(n*1列向量 当前观测输出 p - w*i -k4)
  const Matrixf<dim, 1> &Update(Matrixf<dim, 1> &sampleVector,
                                float actualOutput) {
    // 计算增益向量 觉得了本次参数更新的步长和方向
    gainvector_ =
        (transmatrix_ * sampleVector) /
        (1.0f +
         (sampleVector.Trans() * transmatrix_ * sampleVector)[0][0] / lambda_) /
        lambda_;  // Get gain vector
    paramsvector_ += gainvector_ *
                    (actualOutput - (sampleVector.Trans() *
                                     paramsvector_)[0][0]);  // Get params vector
    transmatrix_ =
        (transmatrix_ - gainvector_ * sampleVector.Trans() * transmatrix_) /
        lambda_;  // Get transferred matrix

    updatecnt_++;
    lastupdate_ = xTaskGetTickCount();
    return paramsvector_;
  }

  /**
   * @brief Set the default regression parameters
   * @param updatedParams
   * @retval None
   */
  void SetParamVector(const Matrixf<dim, 1> &updatedParams) {
    paramsvector_ = updatedParams;
    defaultparamsvector_ = updatedParams;
  }
  /**
   * @brief The getter function of the params vector
   * @param None
   * @retval paramsVector
   */
  constexpr Matrixf<dim, 1> &GetParamsVector() const { return paramsvector_; }

  /**
   * @brief The getter function of the output vector
   * @param None
   * @retval The estimated / filterd output of the RLS module
   */
  const float &GetOutput() const { return output_; }

 private:
  /**
   * @brief Lambda and delta validate check
   * @param None
   * @retval None
   */
  void Validate() const {
    configASSERT(lambda_ >= 0.0f || lambda_ <= 1.0f);
    configASSERT(delta_ > 0);
  }

  uint32_t dimension_;  // Dimension of the RLS space
  float lambda_;        // The forget index
  float delta_;         // Intialized value of the transferred matrix

  TickType_t lastupdate_;  // Last update tick
  uint32_t updatecnt_;     // Total update Count

  /*RLS relvant matrix*/
  Matrixf<dim, dim> transmatrix_;  // Transfer matrix instance
  Matrixf<dim, 1> gainvector_;     // Gain vector for params update
  Matrixf<dim, 1> paramsvector_;   // Params vector
  Matrixf<dim, 1> defaultparamsvector_;
  float output_;  // Estimated / filtered output
};

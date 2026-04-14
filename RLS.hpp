#pragma once

#include <Eigen/Core>

/**
 * @brief 递归最小二乘（RLS）估计器
 * @tparam dim 参数维度
 */
template <uint32_t dim>
class RLS {
 public:
  using ParamVector = Eigen::Matrix<float, dim, 1>;

  RLS() = delete;  // 必须只能带参数的构造函数

  /**
   * @brief 构造 RLS 估计器
   * @param delta_ 初始协方差缩放系数
   * @param lambda_ 遗忘因子
   */
  constexpr RLS(float delta_, float lambda_)
      : dimension_(dim),
        lambda_(lambda_),
        delta_(delta_),
        defaultparamsvector_(ParamVector::Zero()) {
    this->Reset();  // 初始化各个矩阵
  }

  /**
   * @brief 重置估计器状态
   */
  void Reset() {
    transmatrix_ = Eigen::Matrix<float, dim, dim>::Identity() * delta_;
    gainvector_ = ParamVector::Zero();
    paramsvector_ = ParamVector::Zero();
  }

  /**
   * @brief 执行一次 RLS 更新
   * @param sampleVector 输入样本向量
   * @param actualOutput 实际输出
   * @return const ParamVector& 当前参数估计
   */
  const ParamVector& Update(const ParamVector& sample_vector,
                            float actual_output) {
    gainvector_ = (transmatrix_ * sample_vector) /
                  (1.0f + (sample_vector.transpose() * transmatrix_ *
                           sample_vector)(0, 0) /
                              lambda_) /
                  lambda_;
    paramsvector_ +=
        gainvector_ *
        (actual_output - (sample_vector.transpose() * paramsvector_)(0, 0));
    transmatrix_ = (transmatrix_ -
                    gainvector_ * sample_vector.transpose() * transmatrix_) /
                   lambda_;

    return paramsvector_;
  }

  /**
   * @brief 手动设置参数向量
   * @param updatedParams 参数向量
   */
  void SetParamVector(const ParamVector& updated_params) {
    paramsvector_ = updated_params;
    defaultparamsvector_ = updated_params;
  }

 private:
  uint32_t dimension_;
  float lambda_;
  float delta_;

  Eigen::Matrix<float, dim, dim> transmatrix_;
  ParamVector gainvector_;
  ParamVector paramsvector_;
  ParamVector defaultparamsvector_;
};

//pll_deg
#ifndef PLL_DEG_H
#define PLL_DEG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// ===================== PLL 结构体定义 =====================
typedef struct {
    float theta;      // 当前估算角度（度），范围 [0, 360)
    float err;        // 当前角度误差（度），范围 [-180, 180]
    float err_old;    // 上一次的角度误差（度）
    float err_int;    // 速度补偿累积量（度）

    // 可调参数
    float kp;         // 比例增益
    float ki;         // 积分增益
    float out_lim;    // PI输出限幅（度）
    float theta_lim;  // 单次最大角度增量限幅（度），0表示不限幅
} pll_deg_float_t;

// ===================== 外部接口函数 =====================

/**
 * @brief 初始化 PLL
 * @param pll         PLL结构体指针
 * @param init_theta  初始角度（度），建议传入首次霍尔角度，内部自动归一化到 [0,360)
 * @param kp          比例增益
 * @param ki          积分增益
 * @param out_lim     PI输出限幅（度）
 */
void pll_deg_init(pll_deg_float_t *pll, float init_theta, float kp, float ki, float out_lim);

/**
 * @brief 核心更新函数（每控制周期调用一次）
 * @param pll               PLL结构体指针
 * @param hall_theta_deg    霍尔实测角度（度），任意范围，内部会归一化
 * @param theta_acc_deg     前馈角度增量（度），即 速度×周期，可为正或负
 */
void pll_deg_update(pll_deg_float_t *pll, float hall_theta_deg, float theta_acc_deg);

/**
 * @brief 获取滤波后的平滑角度（专门函数）
 * @param pll   PLL结构体指针
 * @return      当前估算角度，范围 [0°, 360°)
 */
float pll_get_theta(const pll_deg_float_t *pll);

/**
 * @brief 设置单步角度增量限幅（可选）
 * @param pll         PLL结构体指针
 * @param theta_lim   最大单步增量（度），0表示不限幅
 */
void pll_set_theta_lim(pll_deg_float_t *pll, float theta_lim);

// ===================== 辅助函数（对外提供，可用于其他模块） =====================

/**
 * @brief 将任意角度归一化到 [0°, 360°) 区间
 * @param angle  任意角度（度）
 * @return       归一化后的角度，范围 [0, 360)
 */
float angle_norm_360(float angle);

/**
 * @brief 计算目标角与当前角的最短误差，结果映射到 [-180°, 180°]
 * @param target  目标角度（度），任意范围
 * @param current 当前角度（度），任意范围
 * @return        最短误差（度），范围 [-180, 180]
 */
float angle_diff_360(float target, float current);

#ifdef __cplusplus
}
#endif

#endif


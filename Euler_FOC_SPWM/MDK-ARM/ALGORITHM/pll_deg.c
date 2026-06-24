#include "pll_deg.h"
#include <math.h>

// ===================== 辅助函数实现 =====================

float angle_norm_360(float angle) {
    // 【防御】如果传入的是 NaN 或 Inf，直接返回 0，避免污染
    if (!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    // 防止因浮点误差导致刚好等于 360°
    if (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

float angle_diff_360(float target, float current) {
    // 【防御】如果传入的是 NaN 或 Inf，直接返回 0，避免污染
    if (!isfinite(target) || !isfinite(current)) {
        return 0.0f;
    }

    float diff = target - current;
    diff = fmodf(diff, 360.0f);
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

// ===================== PLL 核心函数实现 =====================

void pll_deg_init(pll_deg_float_t *pll, float init_theta, float kp, float ki, float out_lim) {
    // 1. 【关键】先将整个结构体内存清零，杜绝随机垃圾值（尤其是 err_old 和 err_int）
    //    如果编译器不支持 {0}，这里手动将所有状态变量清零
    pll->theta   = 0.0f;
    pll->err     = 0.0f;
    pll->err_old = 0.0f;
    pll->err_int = 0.0f;
    
    // 2. 设置有效参数
    pll->kp      = kp;
    pll->ki      = ki;
    pll->out_lim = out_lim;
    pll->theta_lim = 0.0f;   // 默认不限幅

    // 3. 设置初始角度（利用归一化函数，内部自带防 NaN 保护）
    pll->theta = angle_norm_360(init_theta);
}

void pll_deg_update(pll_deg_float_t *pll, float hall_theta_deg, float theta_acc_deg) {
    // =================================================================
    // 第 1 道防线：输入异常拦截（根治 NaN 的最有效手段）
    // 如果霍尔角度或前馈速度是 NaN/Inf，直接丢弃本次更新，保持上一拍角度
    // =================================================================
    if (!isfinite(hall_theta_deg) || !isfinite(theta_acc_deg)) {
        return;  // 静默退出，不污染内部状态
    }

    // ============ 2. 鉴相器 ============
    pll->err = angle_diff_360(hall_theta_deg, pll->theta);
    // 二次保险：如果计算出的误差是 NaN（理论上不会，但以防万一），强制置 0
    if (!isfinite(pll->err)) {
        pll->err = 0.0f;
    }

    // ============ 3. 环路滤波器（增量式 PI） ============
    float out = (pll->err - pll->err_old) * pll->kp + pll->err * pll->ki;
    pll->err_old = pll->err;

    // 再次防御：如果 PI 输出变成 NaN/Inf（可能是参数 Kp/Ki 异常），强制置 0
    if (!isfinite(out)) {
        out = 0.0f;
    }

    // ============ 4. PI 输出限幅 ============
    if (out > pll->out_lim) out = pll->out_lim;
    if (out < -pll->out_lim) out = -pll->out_lim;

    // ============ 5. 误差积分累加器（带饱和保护，防止累积溢出） ============
    pll->err_int += out;

    // 【新增】积分器抗饱和：如果累积量超过合理范围（比如 ±360°），将其截断
    // 防止因长期异常导致 err_int 膨胀到 Inf
    const float ERR_INT_MAX = 360.0f;
    if (pll->err_int > ERR_INT_MAX) pll->err_int = ERR_INT_MAX;
    if (pll->err_int < -ERR_INT_MAX) pll->err_int = -ERR_INT_MAX;

    // ============ 6. 压控振荡器（VCO）：前馈 + 补偿 ============
    float delta_theta = theta_acc_deg + pll->err_int;

    // 第 6.5 道防线：增量限幅，防止单步角度跳变太大
    if (pll->theta_lim > 0.0f) {
        if (delta_theta > pll->theta_lim) delta_theta = pll->theta_lim;
        if (delta_theta < -pll->theta_lim) delta_theta = -pll->theta_lim;
    }

    // ============ 7. 更新角度并归一化 ============
    pll->theta = angle_norm_360(pll->theta + delta_theta);

    // =================================================================
    // 第 8 道防线（终极炸弹拆除）：如果经过以上所有步骤，theta 依然变成了 NaN，
    // 说明可能是浮点硬件寄存器发生了不可预知的错误。
    // 此时强制将角度复位为当前输入的霍尔角度（丢弃滤波值），并清空积分器。
    // =================================================================
    if (!isfinite(pll->theta)) {
        // 直接使用霍尔角度（已检查过非 NaN）强制重建
        pll->theta = angle_norm_360(hall_theta_deg);
        pll->err_int = 0.0f;   // 清空累积误差，重新锁定
    }
}

// ===================== 专门获取角度函数 =====================

float pll_get_theta(const pll_deg_float_t *pll) {
    // 即使内部状态异常，也确保返回一个合法数字
    if (!isfinite(pll->theta)) {
        return 0.0f;
    }
    return pll->theta;
}

// ===================== 设置限幅 =====================

void pll_set_theta_lim(pll_deg_float_t *pll, float theta_lim) {
    if (!isfinite(theta_lim) || theta_lim < 0.0f) {
        pll->theta_lim = 0.0f;  // 非法输入则关闭限幅
    } else {
        pll->theta_lim = theta_lim;
    }
}


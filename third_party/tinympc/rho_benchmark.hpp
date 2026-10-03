#pragma once
#include <cstdint>
#include "types.hpp"

// ⭐⭐⭐ W101（原 F2）：**给成员加默认初始化**，消除未定义行为（UB）。
//
// ⚠️ 原来这四个成员**都没有初值**：
//     struct RhoAdapter { tinytype rho_min; tinytype rho_max; bool clip; bool matrices_initialized; };
//   而 `admm.cpp` 里 `solve()` 会 `if (!adapter->matrices_initialized) { ... }`
//   —— ⚠️ **读的是未初始化内存**（可能是 `true`，于是跳过初始化矩阵那一步 ⇒ 后续算出垃圾）。
//   C++ 标准：读未初始化的标量 = **UB**（不是"恰好为 0"）。
//
// ⭐ 为什么改 `third_party/`（破例，且只此一处）：
//   · 这是 **TinyMPC 自带代码的 bug**，不是设计选择；
//   · 我们的纪律是「不修改上游**行为**」—— 加**初值**不改变任何算法语义，
//     只把"随机值"变成"确定的 0/false"（而 `matrices_initialized=false` 正是
//     作者本意：**还没初始化**）。
//   · 保守起见只动这一处，并在 CMakeLists / 提交里都记明。
struct RhoAdapter {
    tinytype rho_min = 0;
    tinytype rho_max = 0;
    bool clip = false;
    bool matrices_initialized = false;   // ⭐ 默认"未初始化"（作者本意）
    
    // Pre-allocated matrices for formatting
    tinyMatrix A_matrix;
    tinyMatrix z_vector;
    tinyMatrix y_vector;
    tinyMatrix x_decision;
    tinyMatrix P_matrix;
    tinyMatrix q_vector;
    
    // Pre-allocated matrices for residual computation
    tinyMatrix Ax_vector;
    tinyMatrix r_prim_vector;
    tinyMatrix r_dual_vector;
    tinyMatrix Px_vector;
    tinyMatrix ATy_vector;
    
    // Dimensions
    int format_nx;
    int format_nu;
    int format_N;
};

struct RhoBenchmarkResult {
    uint32_t time_us;
    tinytype initial_rho;
    tinytype final_rho;
    tinytype pri_res;
    tinytype dual_res;
    tinytype pri_norm;
    tinytype dual_norm;
};

// Initialize matrices for formatting
void initialize_format_matrices(RhoAdapter* adapter, int nx, int nu, int N);

// Format matrices for residual computation
void format_matrices(
    RhoAdapter* adapter,
    const tinyMatrix& x_prev,
    const tinyMatrix& u_prev,
    const tinyMatrix& v_prev,
    const tinyMatrix& z_prev,
    const tinyMatrix& g_prev,
    const tinyMatrix& y_prev,
    TinyCache* cache,
    TinyWorkspace* work,
    int N
);

// Compute residuals
void compute_residuals(
    RhoAdapter* adapter,
    tinytype* pri_res,
    tinytype* dual_res,
    tinytype* pri_norm,
    tinytype* dual_norm
);

// Predict new rho value
tinytype predict_rho(
    RhoAdapter* adapter,
    tinytype pri_res,
    tinytype dual_res,
    tinytype pri_norm,
    tinytype dual_norm,
    tinytype current_rho
);

// Update matrices using derivatives
void update_matrices_with_derivatives(TinyCache* cache, tinytype new_rho);


// Main benchmark function
void benchmark_rho_adaptation(
    RhoAdapter* adapter,
    const tinyMatrix& x_prev,
    const tinyMatrix& u_prev,
    const tinyMatrix& v_prev,
    const tinyMatrix& z_prev,
    const tinyMatrix& g_prev,
    const tinyMatrix& y_prev,
    TinyCache* cache,
    TinyWorkspace* work,
    int N,
    RhoBenchmarkResult* result
);
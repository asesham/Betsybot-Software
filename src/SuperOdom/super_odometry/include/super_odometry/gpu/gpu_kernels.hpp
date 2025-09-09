#pragma once
#include <cstdint>

namespace so_gpu {

// CUDA curvature (or any per-point score) over a simple index window.
// Inputs are SoA arrays.
void compute_curvature_cuda(const float* x, const float* y, const float* z,
                            int32_t n_points,
                            int window,           // e.g., 5
                            float* out_curvatures // n_points
                            );

} // namespace so_gpu

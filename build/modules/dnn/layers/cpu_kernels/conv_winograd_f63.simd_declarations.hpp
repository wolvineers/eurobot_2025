#define CV_CPU_SIMD_FILENAME "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/modules/dnn/src/layers/cpu_kernels/conv_winograd_f63.simd.hpp"
#define CV_CPU_DISPATCH_MODE AVX
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODE AVX2
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODE NEON_FP16
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODES_ALL NEON_FP16, AVX2, AVX, BASELINE

#undef CV_CPU_SIMD_FILENAME

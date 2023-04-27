/* Convenience functions for Operations.
 */

#pragma once

// STL
#include <assert.h>
#include <utility>
#include <cstddef>

namespace ovm::ops
{

// allow cross-compilation between CUDA and Host
#ifdef __CUDACC__
#define CUDA_HOSTDEV __host__ __device__
#else
#define CUDA_HOSTDEV
#endif

// convenience transforms from Eigen coordinates (r,c) to OpenVDB 2D index (i,j)
// ij = R(pi/2) * (rc - Orc)
CUDA_HOSTDEV static inline std::pair<int,int> rc_to_idx(const size_t r, const size_t c, const int Or, const int Oc)
{
  return {c - Oc, Or - r};
}

// convenience transforms from OpenVDB 2D index (i,j) to Eigen coordinates (r,c)
// rc = R(-pi/2) * (ij - Oij)
CUDA_HOSTDEV static inline std::pair<size_t,size_t> idx_to_rc(const int i, const int j, const int Oi, const int Oj)
{
  // sanity checks
  assert(Oj >= j && i >= Oi);
  return {Oj - j, i - Oi};
}

} // namespace ovm::ops
// Repeatable QP snapshot I/O for solver comparisons.
#ifndef LU_OMMPC__QP_DATASET_HPP_
#define LU_OMMPC__QP_DATASET_HPP_

#include <cstdint>
#include <fstream>
#include <string>

#include "lu_ommpc/types.hpp"

namespace lu_ommpc
{

struct QPSnapshot
{
  uint64_t timestamp_us{0};
  QPProblem problem;
  Eigen::VectorXd warm_start;
  State state;
  ReferenceHorizon reference;
  bool has_mpc_input{false};
};

class QPDatasetWriter
{
public:
  explicit QPDatasetWriter(const std::string & path);
  bool good() const;
  bool write(const QPSnapshot & snapshot);

private:
  std::ofstream stream_;
};

class QPDatasetReader
{
public:
  explicit QPDatasetReader(const std::string & path);
  bool good() const;
  bool read(QPSnapshot & snapshot);

private:
  std::ifstream stream_;
};

}  // namespace lu_ommpc

#endif  // LU_OMMPC__QP_DATASET_HPP_

// Repeatable QP snapshot I/O for solver comparisons.
#include "lu_ommpc/qp_dataset.hpp"

#include <array>
#include <cstring>
#include <limits>

namespace lu_ommpc
{
namespace
{

constexpr std::array<char, 8> kMagic{{'L', 'U', 'Q', 'P', 'D', 'S', '2', '\0'}};
constexpr uint32_t kRecordMarker = 0x51505232U;

template<typename T>
bool writeScalar(std::ofstream & stream, const T & value)
{
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return stream.good();
}

template<typename T>
bool readScalar(std::ifstream & stream, T & value)
{
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  return stream.good();
}

bool writeMatrix(std::ofstream & stream, const Eigen::MatrixXd & matrix)
{
  stream.write(
    reinterpret_cast<const char *>(matrix.data()),
    static_cast<std::streamsize>(matrix.size() * sizeof(double)));
  return stream.good();
}

bool writeVector(std::ofstream & stream, const Eigen::VectorXd & vector)
{
  stream.write(
    reinterpret_cast<const char *>(vector.data()),
    static_cast<std::streamsize>(vector.size() * sizeof(double)));
  return stream.good();
}

bool readMatrix(std::ifstream & stream, Eigen::MatrixXd & matrix)
{
  stream.read(
    reinterpret_cast<char *>(matrix.data()),
    static_cast<std::streamsize>(matrix.size() * sizeof(double)));
  return stream.good();
}

bool readVector(std::ifstream & stream, Eigen::VectorXd & vector)
{
  stream.read(
    reinterpret_cast<char *>(vector.data()),
    static_cast<std::streamsize>(vector.size() * sizeof(double)));
  return stream.good();
}

template<typename Derived>
bool writeEigen(std::ofstream & stream, const Eigen::MatrixBase<Derived> & value)
{
  stream.write(
    reinterpret_cast<const char *>(value.derived().data()),
    static_cast<std::streamsize>(value.size() * sizeof(double)));
  return stream.good();
}

template<typename Derived>
bool readEigen(std::ifstream & stream, Eigen::MatrixBase<Derived> const & value_base)
{
  auto & value = const_cast<Derived &>(value_base.derived());
  stream.read(
    reinterpret_cast<char *>(value.data()),
    static_cast<std::streamsize>(value.size() * sizeof(double)));
  return stream.good();
}

}  // namespace

QPDatasetWriter::QPDatasetWriter(const std::string & path)
: stream_(path, std::ios::binary | std::ios::trunc)
{
  if (stream_) {
    stream_.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
  }
}

bool QPDatasetWriter::good() const
{
  return stream_.good();
}

bool QPDatasetWriter::write(const QPSnapshot & snapshot)
{
  const auto & p = snapshot.problem;
  if (!stream_ || p.H.rows() != p.H.cols() || p.H.rows() != p.g.size() ||
    p.A.cols() != p.g.size() || p.A.rows() != p.lower.size() ||
    p.lower.size() != p.upper.size())
  {
    return false;
  }
  const int32_t variables = static_cast<int32_t>(p.g.size());
  const int32_t constraints = static_cast<int32_t>(p.A.rows());
  const int32_t warm_size = static_cast<int32_t>(snapshot.warm_start.size());
  const uint32_t flags = (p.ocp.valid() ? 1U : 0U) | (snapshot.has_mpc_input ? 2U : 0U);
  const int32_t stages = p.ocp.valid() ? static_cast<int32_t>(p.ocp.A.size()) : 0;
  const int32_t knots = snapshot.has_mpc_input ?
    static_cast<int32_t>(snapshot.reference.size()) : 0;
  bool ok = writeScalar(stream_, kRecordMarker) &&
         writeScalar(stream_, snapshot.timestamp_us) &&
         writeScalar(stream_, variables) && writeScalar(stream_, constraints) &&
         writeScalar(stream_, warm_size) && writeMatrix(stream_, p.H) &&
         writeVector(stream_, p.g) && writeMatrix(stream_, p.A) &&
         writeVector(stream_, p.lower) && writeVector(stream_, p.upper) &&
         writeVector(stream_, snapshot.warm_start) && writeScalar(stream_, flags) &&
         writeScalar(stream_, stages) && writeScalar(stream_, knots);
  if (!ok) {return false;}
  if (p.ocp.valid()) {
    ok = writeEigen(stream_, p.ocp.x0) && writeEigen(stream_, p.ocp.Q) &&
      writeEigen(stream_, p.ocp.P) && writeEigen(stream_, p.ocp.R);
    for (int k = 0; ok && k < stages; ++k) {
      ok = writeEigen(stream_, p.ocp.A[k]) && writeEigen(stream_, p.ocp.B[k]) &&
        writeEigen(stream_, p.ocp.lower_u[k]) && writeEigen(stream_, p.ocp.upper_u[k]);
    }
  }
  if (ok && snapshot.has_mpc_input) {
    ok = writeEigen(stream_, snapshot.state.position) &&
      writeEigen(stream_, snapshot.state.velocity) && writeEigen(stream_, snapshot.state.rotation);
    for (const auto & knot : snapshot.reference) {
      ok = ok && writeEigen(stream_, knot.state.position) &&
        writeEigen(stream_, knot.state.velocity) && writeEigen(stream_, knot.state.rotation);
      const Eigen::Vector4d input = knot.input.vector();
      ok = ok && writeEigen(stream_, input);
    }
  }
  return ok;
}

QPDatasetReader::QPDatasetReader(const std::string & path)
: stream_(path, std::ios::binary)
{
  std::array<char, 8> magic{};
  if (stream_) {
    stream_.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (magic != kMagic) {
      stream_.setstate(std::ios::failbit);
    }
  }
}

bool QPDatasetReader::good() const
{
  return stream_.good();
}

bool QPDatasetReader::read(QPSnapshot & snapshot)
{
  uint32_t marker = 0;
  if (!readScalar(stream_, marker)) {
    return false;
  }
  int32_t variables = 0;
  int32_t constraints = 0;
  int32_t warm_size = 0;
  if (marker != kRecordMarker || !readScalar(stream_, snapshot.timestamp_us) ||
    !readScalar(stream_, variables) || !readScalar(stream_, constraints) ||
    !readScalar(stream_, warm_size) || variables <= 0 || constraints <= 0 ||
    warm_size < 0 || variables > 100000 || constraints > 100000 || warm_size > 100000)
  {
    stream_.setstate(std::ios::failbit);
    return false;
  }
  auto & p = snapshot.problem;
  p.H.resize(variables, variables);
  p.g.resize(variables);
  p.A.resize(constraints, variables);
  p.lower.resize(constraints);
  p.upper.resize(constraints);
  snapshot.warm_start.resize(warm_size);
  if (!(readMatrix(stream_, p.H) && readVector(stream_, p.g) &&
         readMatrix(stream_, p.A) && readVector(stream_, p.lower) &&
         readVector(stream_, p.upper) && readVector(stream_, snapshot.warm_start))) {
    return false;
  }
  uint32_t flags = 0;
  int32_t stages = 0;
  int32_t knots = 0;
  if (!readScalar(stream_, flags) || !readScalar(stream_, stages) ||
    !readScalar(stream_, knots) || stages < 0 || stages > 10000 ||
    knots < 0 || knots > 10001)
  {
    return false;
  }
  if ((flags & 1U) != 0U) {
    p.ocp.A.resize(stages); p.ocp.B.resize(stages);
    p.ocp.lower_u.resize(stages); p.ocp.upper_u.resize(stages);
    if (!(readEigen(stream_, p.ocp.x0) && readEigen(stream_, p.ocp.Q) &&
      readEigen(stream_, p.ocp.P) && readEigen(stream_, p.ocp.R))) {return false;}
    for (int k = 0; k < stages; ++k) {
      if (!(readEigen(stream_, p.ocp.A[k]) && readEigen(stream_, p.ocp.B[k]) &&
        readEigen(stream_, p.ocp.lower_u[k]) && readEigen(stream_, p.ocp.upper_u[k]))) {
        return false;
      }
    }
  }
  snapshot.has_mpc_input = (flags & 2U) != 0U;
  snapshot.reference.resize(static_cast<std::size_t>(knots));
  if (snapshot.has_mpc_input) {
    if (!(readEigen(stream_, snapshot.state.position) &&
      readEigen(stream_, snapshot.state.velocity) && readEigen(stream_, snapshot.state.rotation))) {
      return false;
    }
    for (auto & knot : snapshot.reference) {
      Eigen::Vector4d input;
      if (!(readEigen(stream_, knot.state.position) && readEigen(stream_, knot.state.velocity) &&
        readEigen(stream_, knot.state.rotation) && readEigen(stream_, input))) {return false;}
      knot.input.thrust_acceleration = input[0];
      knot.input.body_rate = input.tail<3>();
    }
  }
  return true;
}

}  // namespace lu_ommpc

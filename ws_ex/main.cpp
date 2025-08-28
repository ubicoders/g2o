#include <iostream>
#include <iomanip>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

// SBA types (poses/points/projection edges)
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using std::cout;
using std::endl;

static inline double sqr(double x) { return x * x; }

void showCameraPoses(g2o::SparseOptimizer& opt, int firstId, int N) {
  for (int i = firstId; i < firstId + N; ++i) {
    auto* v = dynamic_cast<g2o::VertexSE3Expmap*>(opt.vertex(i));
    if (!v) continue;
    const auto& se3 = v->estimate();
    Eigen::Vector3d t = se3.translation();
    cout << "camera pose at " << i << ": [" << t.transpose() << "]\n";
  }
}

double calcSSE_3Dpoints(g2o::SparseOptimizer& opt, int firstPointId,
                        const std::vector<Eigen::Vector3d>& wPts)
{
  double sse = 0.0;
  for (int i = 0; i < (int)wPts.size(); ++i) {
    auto* v = dynamic_cast<g2o::VertexPointXYZ*>(opt.vertex(firstPointId + i));
    if (!v) continue;
    Eigen::Vector3d err = v->estimate() - wPts[i];
    sse += err.squaredNorm();
  }
  return sse;
}

void showWpts(g2o::SparseOptimizer& opt, int firstPointId, int N) {
  for (int i = 0; i < N; ++i) {
    auto* v = dynamic_cast<g2o::VertexPointXYZ*>(opt.vertex(firstPointId + i));
    if (!v) continue;
    cout << "guessed wPt at " << i << ": [" << v->estimate().transpose() << "]\n";
  }
}

// Simple pinhole projection to synthesize measurements
Eigen::Vector2d projectUV(const Eigen::Vector3d& Pw,
                          const g2o::SE3Quat& cam,
                          double fx, double fy, double cx, double cy)
{
  Eigen::Vector3d Pc = cam.map(Pw);
  const double z = Pc.z();
  return Eigen::Vector2d(fx * Pc.x() / z + cx, fy * Pc.y() / z + cy);
}

int main() {
  std::mt19937 rng(42);
  std::normal_distribution<double> noise_uv(0.0, 1.0);    // pixel noise
  std::normal_distribution<double> noise_xyz(0.0, 1e-1);  // init noise

  // Intrinsics (match your Python)
  const double f = 200.0, p = 256.0;
  const double fx = f, fy = f, cx = p, cy = p;

  // Ground-truth world points
  std::vector<Eigen::Vector3d> wPts{
    { 0.0,  0.0, 10.0},
    {-1.0,  3.0, 30.0},
    { 2.0,  2.0, 37.2}
  };

  // Optimizer (Levenberg + CHOLMOD)
  using BlockSolver = g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >;
  auto linearSolver = std::make_unique< g2o::LinearSolverCholmod<BlockSolver::PoseMatrixType> >();
  auto blockSolver  = std::make_unique<BlockSolver>(std::move(linearSolver));
  auto algorithm    = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(false);

  // Camera parameters block (fx ~ fy, principal point, baseline=0 for mono)
  auto* camParams = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0.0);
  camParams->setId(0);
  optimizer.addParameter(camParams);

  // Create 10 poses: first half along +x, second half along +y
  const int num_pose = 10;
  for (int i = 0; i < num_pose/2; ++i) {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t((i-2)*10.0, 0.0, 0.0);
    auto* v = new g2o::VertexSE3Expmap();
    v->setId(i);
    v->setEstimate(g2o::SE3Quat(q, t));
    if (i < 2) v->setFixed(true);  // fix a couple like Python
    optimizer.addVertex(v);
  }
  for (int i = num_pose/2; i < num_pose; ++i) {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t(0.0, (i - num_pose/2 - 2) * 10.0, 0.0);
    auto* v = new g2o::VertexSE3Expmap();
    v->setId(i);
    v->setEstimate(g2o::SE3Quat(q, t));
    optimizer.addVertex(v);
  }

  // Add 3 landmark vertices with noisy init (VertexPointXYZ on newer g2o)
  const int firstPointId = num_pose;
  for (int i = 0; i < (int)wPts.size(); ++i) {
    auto* vp = new g2o::VertexPointXYZ();
    vp->setId(firstPointId + i);
    vp->setMarginalized(true);
    Eigen::Vector3d init = wPts[i];
    init.x() += noise_xyz(rng);
    init.y() += noise_xyz(rng);
    init.z() += noise_xyz(rng);
    vp->setEstimate(init);
    optimizer.addVertex(vp);
  }

  // Create monocular reprojection edges for every (pose, point)
  for (int pid = 0; pid < (int)wPts.size(); ++pid) {
    for (int camId = 0; camId < num_pose; ++camId) {
      auto* vPose = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(camId));
      Eigen::Vector2d z = projectUV(wPts[pid], vPose->estimate(), fx, fy, cx, cy);
      if (pid > 1) { // add noise to last point’s measurements (like Python)
        z.x() += noise_uv(rng);
        z.y() += noise_uv(rng);
      }

      auto* e = new g2o::EdgeProjectXYZ2UV();
      e->setVertex(0, optimizer.vertex(firstPointId + pid)); // point
      e->setVertex(1, optimizer.vertex(camId));              // pose
      e->setMeasurement(z);
      e->information() = Eigen::Matrix2d::Identity();
      e->setParameterId(0, 0); // attach intrinsics

      optimizer.addEdge(e);
    }
  }

  // Before optimization
  double sse0 = calcSSE_3Dpoints(optimizer, firstPointId, wPts);
  cout << std::fixed << std::setprecision(6);
  cout << "\nRMSE (3D landmark error):\n";
  cout << "before optimization: " << std::sqrt(sse0 / wPts.size()) << "\n";
  showCameraPoses(optimizer, 0, num_pose);
  showWpts(optimizer, firstPointId, (int)wPts.size());

  // Optimize
  cout << "\nPerforming full BA:\n";
  optimizer.initializeOptimization();
  optimizer.optimize(100);

  // After optimization
  double sse1 = calcSSE_3Dpoints(optimizer, firstPointId, wPts);
  cout << "\nRMSE (3D landmark error):\n";
  cout << "after  optimization: " << std::sqrt(sse1 / wPts.size()) << "\n";
  showCameraPoses(optimizer, 0, num_pose);
  showWpts(optimizer, firstPointId, (int)wPts.size());

  return 0;
}

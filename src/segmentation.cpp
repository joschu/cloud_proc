#include "segmentation.h"
#include "cloud_ops.h"
#include "table.h"
#include "utils_pcl.h"

using namespace std;
using namespace Eigen;

MatrixXf extractRows(const MatrixXf& in, const VectorXb& mask) {
  assert(mask.size() == in.rows());
  int rowsOut = mask.cast<int>().sum();
  MatrixXf out(rowsOut, in.cols());
//  cout << mask.transpose() << endl;

  int outRow = 0;
  for (int i=0; i < mask.size(); ++i) {
    if (mask(i)) {
      out.row(outRow) = in.row(i);
      ++outRow;
    }
  }
  return out;
}

vector<int> flatnonzero(const VectorXb& in) {
  vector<int> out;
  int n = in.sum();
  out.reserve(n);
  for (int i=0; i < in.size(); ++i) {
    out.push_back(i);
  }
  return out;
}



vector<ColorCloudPtr> segmentPointCloud(ColorCloudPtr in) {

  in = downsampleCloud(in, .01);

  float height = getTableHeight(in);

  MatrixXf xyz = toEigenMatrix(in);
  VectorXf z = xyz.col(2);
  VectorXf zpred(xyz.rows()); zpred.setConstant(height);

  int nIter=3;

  for (int i=0; i < nIter; ++i) {
    printf("iteration %i\n", i);
    printf("%i %i\n", z.size(), zpred.size());
    VectorXb inlierMask = (z.array() - zpred.array()).abs() < .005;

    MatrixXf inliers = extractRows(xyz, inlierMask);
    // 1 x y x^2 y^2 xy
    MatrixXf quad(inliers.rows(), 6);
    quad.col(0).setOnes();
    quad.col(1) = inliers.col(0);
    quad.col(2) = inliers.col(1);
    quad.col(3) = inliers.col(0).cwiseProduct(inliers.col(0));
    quad.col(4) = inliers.col(1).cwiseProduct(inliers.col(1));
    quad.col(5) = inliers.col(0).cwiseProduct(inliers.col(1));
    VectorXf coeffs = quad.jacobiSvd(ComputeThinU | ComputeThinV).solve(extractRows(z,inlierMask));
    cout << "coeffs: " << coeffs.transpose() << endl;;
    zpred = quad * coeffs;
  }

  VectorXb aboveTableMask = (z.array() - zpred.array()) > .01;
  ColorCloudPtr aboveTableCloud = maskCloud(in, aboveTableMask);
  vector<vector<int> > clusterInds = findClusters(aboveTableCloud, .02, 10);
  vector<ColorCloudPtr> out;
  for (int i=0; i < clusterInds.size(); ++i) out.push_back(extractInds(in, clusterInds[i]));
  return out;
}

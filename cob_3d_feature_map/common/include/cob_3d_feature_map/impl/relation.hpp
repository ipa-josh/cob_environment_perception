/*
 * relation.hpp
 *
 *  Created on: 23.03.2013
 *      Author: josh
 */


template<int n>
bool RelationDistribution<n>::compute()
{
  if(size_ < n)
  {
    std::cerr<<"RelationDistribution: size < n / "<<size_<<"<"<<n<<std::endl;
    return (false);
  }

  Eigen::SelfAdjointEigenSolver<MatrixU> evd (co_/size_);
  // Organize Eigenvectors and eigenvalues in ascendent order
  for (int i = 0; i < 3; ++i)
  {
    eigenvalues_[i] = evd.eigenvalues () [2-i];
    eigenvectors_.col (i) = evd.eigenvectors ().col (2-i);
  }

  computed_ = true;
  return (true);
}

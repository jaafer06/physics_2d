#pragma once
#include <iostream>
#include <Eigen/Dense>

namespace utils {
	namespace EigenUtils {
		template<typename Type, int n, int m>
		std::ostream& operator<<(std::ostream& os, Eigen::Matrix<Type, n, m>& matrix) {
			for (unsigned int i = 0; i < n; ++i) {
				for (unsigned int j = 0; j < m; ++j) {
					os << matrix(i, j) << "\t";
				};
				os << std::endl;
			}
			return os;
		}

		template<typename Type, int n>
		std::ostream& operator<<(std::ostream& os, Eigen::Matrix<Type, n, 1>& vector) {
			for (unsigned int i = 0; i < n; ++i) {
				os << vector[i]<< "\t";
			}
			os << std::endl;
			return os;
		}

	}
};
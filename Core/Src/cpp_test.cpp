#include "cpp_test.h"
#include <vector>
#include "Eigen/Core"

void cpp_test() {
    Eigen::Matrix<int, 3, 3> {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
    
    std::vector<int> v;
    v.push_back(42);
    v.pop_back();
}
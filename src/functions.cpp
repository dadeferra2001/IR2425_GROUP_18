#include "assignment1/functions.h"

std::string vectorToCSV(const std::vector<int>& vec) {
    std::ostringstream oss;

    std::vector<int> sorted_vec = vec;
    std::sort(sorted_vec.begin(), sorted_vec.end());
    
    for (size_t i = 0; i < sorted_vec.size(); ++i) {
        oss << sorted_vec[i];
        if (i < sorted_vec.size() - 1) {
            oss << ",";
        }
    }
    return oss.str();
}

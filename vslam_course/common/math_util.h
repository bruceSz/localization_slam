#pragma once
#include <vector>
namespace zs {

// ssd or sse sum of squared error/difference
template<typename T>
double ssd(const std::vector<T>& src, const std::vector<T>& ref) {
    assert(src.size() == ref.size());

    double sum = 0;
    for(int i=0; i<src.size(); i++) {
        double d = src[i] - ref[i];
        sum += d*d;

    }
    return sum;
}

} // namespace zs
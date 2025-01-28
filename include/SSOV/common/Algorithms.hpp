#pragma once

#include <vector>

namespace ssov {

template <typename T> int insert_index(const std::vector<T>& array, T element) {
    int start = 0;
    int end = array.size() - 1;

    while (start <= end) {
        int mid = (start + end) / 2;
        if (element == array[mid]) {
            return mid;
        } else if (element < array[mid]) {
            end = mid - 1;
        } else {
            start = mid + 1;
        }
    }
    return end + 1;
}

}
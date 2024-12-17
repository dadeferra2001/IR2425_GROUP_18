#ifndef ASSIGNMENT1_FUNCTIONS_H
#define ASSIGNMENT1_FUNCTIONS_H

#include <vector>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>

template <typename K, typename V>
std::vector<K> getKeys(const std::map<K, V>& inputMap) {
    std::vector<K> keys;
    for (const auto& pair : inputMap) {
        keys.push_back(pair.first);
    }
    return keys;
}

template <typename K, typename V>
std::vector<V> getValues(const std::map<K, V>& inputMap) {
    std::vector<V> values;
    for (const auto& pair : inputMap) {
        values.push_back(pair.second);
    }
    return values;
}

// Function to convert a vector to a CSV string
std::string vectorToCSV(const std::vector<int>& vec);

#endif // ASSIGNMENT1_FUNCTIONS_H

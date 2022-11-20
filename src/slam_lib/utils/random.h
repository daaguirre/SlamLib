#include <vector>

namespace slam
{

template <class T>
std::vector<T> random_selection(const std::vector<T>& data, size_t num_random)
{
    std::vector<T> selected;
    selected.reserve(num_random);
    const size_t n = data.size();
    while (num_random--)
    {
        auto it = data.begin();
        std::advance(it, rand() % n);
        selected.push_back(*it);
    }
    return selected;
}

}
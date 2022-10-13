/**
 * @file map_reader.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SRC_MAP_READER_H_
#define __SRC_MAP_READER_H_

#include <string>

#include "slam_lib/mapping/occupancy_grid.h"

namespace slam
{

template <typename FloatT>
class MapReader
{
    static const std::string RESOLUTION_STRING;
    static const std::string MAP_SIZE_STRING;

public:
    MapReader() = default;

    typename OccupancyGrid<FloatT>::Ptr read_map(const std::string &file_path);

private:
    FloatT get_resolution(std::ifstream &file) const;
    std::tuple<size_t, size_t> get_map_size(std::ifstream &file) const;

    void fill_probability_map(std::ifstream &file, typename OccupancyGrid<FloatT>::Map &map) const;
};

}  // namespace slam

#endif  // __SRC_MAP_READER_H_

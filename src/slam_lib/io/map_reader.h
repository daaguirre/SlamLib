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

class MapReader
{
    static const std::string RESOLUTION_STRING;
    static const std::string MAP_SIZE_STRING;

public:
    MapReader() = default;

    OccupancyGrid read_map(const std::string &file_path);

private:
    float get_resolution(std::ifstream &file) const;
    std::tuple<size_t, size_t> get_map_size(std::ifstream &file) const;
    void fill_probability_map(std::ifstream &file, OccupancyGrid::Map &map) const;
};

}

#endif  // __SRC_MAP_READER_H_

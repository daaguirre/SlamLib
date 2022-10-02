
#include "map_reader.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace slam
{

const std::string MapReader::RESOLUTION_STRING = "robot_specifications->resolution";
const std::string MapReader::MAP_SIZE_STRING = "global_map[0]:";

OccupancyGrid MapReader::read_map(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (file.fail())
    {
        std::stringstream error_stream;
        error_stream << "Input file " << file_path << " not found\n";
        throw std::invalid_argument(error_stream.str());
    }
    std::string map_name = std::filesystem::path(file_path).replace_extension().filename();
    float resolution = get_resolution(file);
    auto [size_x, size_y] = get_map_size(file);
    OccupancyGrid occupancy_grid(size_x, size_y, resolution, map_name);
    fill_probability_map(file, occupancy_grid.m_map);

    return occupancy_grid;
}

float MapReader::get_resolution(std::ifstream &file) const
{
    float resolution = 0.0f;
    std::string line;
    while (std::getline(file, line))
    {
        const auto str_pos = line.find(RESOLUTION_STRING);
        if (str_pos != std::string::npos)
        {
            size_t start_pos = str_pos + RESOLUTION_STRING.size();
            std::istringstream iss(line.substr(start_pos));
            if (!(iss >> resolution))
            {
                throw std::runtime_error("Invalid file format. Resolution not found.");
            }
            break;
        }
    }

    return resolution*0.01;
}

std::tuple<size_t, size_t> MapReader::get_map_size(std::ifstream &file) const
{
    std::string line;
    size_t size_x = 0, size_y = 0;
    while (std::getline(file, line))
    {
        const auto str_pos = line.find(MAP_SIZE_STRING);
        if (str_pos != std::string::npos)
        {
            size_t start_pos = str_pos + MAP_SIZE_STRING.size();
            std::istringstream iss(line.substr(start_pos));

            if (!(iss >> size_x >> size_y))
            {
                throw std::runtime_error("Invalid file format. Map size not found.");
            }
            break;
        }
    }

    return {size_x, size_y};
}

void MapReader::fill_probability_map(
    std::ifstream &file,
    OccupancyGrid::Map &map) const
{
    size_t x_idx = 0, y_idx = 0;
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        float value = 0.0f;
        while ((iss >> value))
        {
            map(y_idx, x_idx) = value;
            ++x_idx;
        }
        if (x_idx != static_cast<size_t>(map.cols()))
        {
            throw std::runtime_error("Invalid file. Data does not match specified dimensions.");
        }

        ++y_idx;
        x_idx = 0;
    }

    if (y_idx != static_cast<size_t>(map.rows()))
    {
        throw std::runtime_error("Invalid file. Data does not match specified dimensions.");
    }
}

}
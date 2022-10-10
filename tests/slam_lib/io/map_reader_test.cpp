#include "map_reader_test.h"

#include <slam_lib/io/map_reader.h>

TEST_F(MapReaderTest, should_read_file)
{
    using FloatT = float;
    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT> wean_map = map_reader.read_map(wean_map_path);
    
    const std::string expected_name = "wean";
    const FloatT expected_resolution = 0.1f;
    ASSERT_EQ(expected_name, wean_map.name());
    ASSERT_FLOAT_EQ(expected_resolution, wean_map.resolution());
    // const slam::OccupancyGrid::Map& prob_map = wean_map.get_probability_grid();
    ASSERT_EQ(800, wean_map.width());
    ASSERT_EQ(800, wean_map.height());
}
#include "../librealsense/include/librealsense2/rs.hpp"
#include <iostream>

int main() try
{
    rs2::context ctx;
    rs2::device dev = ctx.query_devices().front();

    rs2::stream_profile depth_stream;
    rs2::stream_profile gyro_stream;
    rs2::stream_profile accel_stream;

    // Loop through sensors and their stream profiles
    for (auto&& sensor : dev.query_sensors())
    {
        for (auto&& profile : sensor.get_stream_profiles())
        {
            if (profile.stream_type() == RS2_STREAM_DEPTH)
                depth_stream = profile;
            else if (profile.stream_type() == RS2_STREAM_GYRO)
                gyro_stream = profile;
            else if (profile.stream_type() == RS2_STREAM_ACCEL)
                accel_stream = profile;
        }
    }

    if (depth_stream && gyro_stream)
    {
        auto extr = gyro_stream.get_extrinsics_to(depth_stream);
        std::cout << "Gyro → Depth extrinsics:\n";
        std::cout << "  Rotation matrix:\n";
        for (int i = 0; i < 9; i++)
        {
            std::cout << extr.rotation[i] << " ";
            if ((i + 1) % 3 == 0) std::cout << std::endl;
        }
        std::cout << "  Translation (m): ["
                  << extr.translation[0] << ", "
                  << extr.translation[1] << ", "
                  << extr.translation[2] << "]\n";
    }

    if (depth_stream && accel_stream)
    {
        auto extr = accel_stream.get_extrinsics_to(depth_stream);
        std::cout << "Accel → Depth extrinsics:\n";
        std::cout << "  Rotation matrix:\n";
        for (int i = 0; i < 9; i++)
        {
            std::cout << extr.rotation[i] << " ";
            if ((i + 1) % 3 == 0) std::cout << std::endl;
        }
        std::cout << "  Translation (m): ["
                  << extr.translation[0] << ", "
                  << extr.translation[1] << ", "
                  << extr.translation[2] << "]\n";
    }

    return 0;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

# Kalman Sensor Fusion 

TODO: finish the readme

This is a simple project to learn how to use modern CMake for C++ development. The idea is to create simple sensor fusion library that uses several external libraries. The fusion library should be properly formatted, documented and tested. In addition, the library should be tested in a Docker environment in different configurations 

## Learning objectives

- Create docker environment for C++ development
- Add a header-only library using git submodule (nlohmann::json)
- Add a library using git submodule (glog)
- Add a library using fetch content (gtest)
- Add a library using find package (boost)
- Add a library using Conan (eigen)
- Add a library using vcpkg (opencv) **Takes too long for some reason**
- Build code with several build configurations: debug, release, gui etc
- Run code checks: Clang-Format, Clang-Tidy, Cppcheck, Cpplint 
- Run code in debug mode with dgb
- Create and run unit and integration tests
- Create Doxygen documentation

## Sensor Fusion

More about the sensor fusion [here](./docs/sensor_fusion.md) 

## How to run

### Environment

`docker-compose up`

### VSCode

### Command line 

`docker exec -it cpp_workspace bash`
`cd /workspace/build && conan install .. --profile=../conan_profiles/eigen_profile`

### Usage examples
`./build/bin/main --config /workspace/configs/config.json --input /workspace/data/test_data_2.json --output /workspace/data/`

## References
- The idea is taken from [CarND-Extended-Kalman-Filter-Project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)
- Project structure is composed from [Canonical Project Structure](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2018/p1204r0.html) and [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/chapters/basics/structure.html)
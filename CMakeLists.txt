cmake_minimum_required(VERSION 3.10)
project(DifferentialMpc)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找依赖库Eigen
find_package(Eigen3 REQUIRED)
# 寻找依赖库osqp和OsqpEigen
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PythonLibs REQUIRED)

# 添加include目录
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OSQP_INCLUDE_DIRS}
    ${OSQPEIGEN_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PYTHON_INCLUDE_DIRS}
    #/usr/include/python3.10
)

# 指定编译输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 指定源文件
set(QuinticPolynomial 
    src/QuinticPolynomialInterpolation/main.cpp
)
set(CURVATUREBYTHREEPOINTS 
    src/CurvatureByThreePoints/main.cpp
)

# 添加源码目录
add_executable(quinticpolynomial_ ${QuinticPolynomial})
add_executable(curvaturebythreepoints ${CURVATUREBYTHREEPOINTS})

# 链接依赖库
target_link_libraries(quinticpolynomial_ 
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
    ${PYTHON_LIBRARIES}
    python3.10.so
    pthread
    dl
)

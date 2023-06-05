# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (bev2cloud)

# 查找PCL库
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find Open3D
#find_package(Open3D REQUIRED)
#include_directories(${Open3D_INCLUDE_DIRS})
#link_directories(${Open3D_LIBRARY_DIRS})
find_package(g2o REQUIRED)
include_directories(${g2o_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

# find_package(yaml-cpp REQUIRED)
# include_directories(${YAML_CPP_INCLUDE_DIR})


# 查找目录下的所有源文件和头文件
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/yaml-cpp)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRCS)
FILE(GLOB INC_PATH ${PROJECT_SOURCE_DIR}/include/*.h)
LIST(APPEND INCS ${INC_PATH} )
link_directories(
    ${PROJECT_SOURCE_DIR}/lib
)
link_directories(/usr/local/lib)

# Add your source files here
set(SOURCES main.cpp)

# 指定生成目标
add_executable(bev2cloud main.cpp ${INCS} ${SRCS} )
target_link_libraries (bev2cloud  g2o_types_sba  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${g2o_LIBS} yaml-cpp)

target_link_libraries(bev2cloud g2o_core g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 fmt)

#target_link_libraries (bev2cloud ${PCL_LIBRARIES} ${Open3D_LIBRARIES} ${OpenCV_LIBS})

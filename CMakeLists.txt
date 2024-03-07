cmake_minimum_required(VERSION 3.5)
project(third_challenge_natsume)

find_package(ament_cmake_auto REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/third_challenge_natsume.cpp
  src/third_challenge_node_natsume.cpp)

# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()

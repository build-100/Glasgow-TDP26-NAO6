cmake_minimum_required(VERSION 3.0.2)
project(nao6_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md LICENSE.upstream source_manifest.json adaptation_report.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

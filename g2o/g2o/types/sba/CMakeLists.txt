ADD_LIBRARY(${LIB_PREFIX}types_sba ${G2O_LIB_TYPE}
  types_sba.h     types_six_dof_expmap.h
  types_sba.cpp   types_six_dof_expmap.cpp
  g2o_types_sba_api.h
)

TARGET_LINK_LIBRARIES(${LIB_PREFIX}types_sba ${LIB_PREFIX}core ${LIB_PREFIX}types_slam3d)

INSTALL(TARGETS ${LIB_PREFIX}types_sba
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/sba)

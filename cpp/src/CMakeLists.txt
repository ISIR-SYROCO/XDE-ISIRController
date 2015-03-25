include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${PC_EIGEN_INCLUDE_DIRS})
#include_directories(/usr/include/eigen3)
#include_directories(/usr/include/eigen3/unsupported)

include_directories(
    ${OROCOS-RTT_INCLUDE_DIR}
    ${PC_XDECORE_INCLUDE_DIRS}   
    ${PC_OCL_INCLUDE_DIRS}
    ${PC_OCL_DEPLOY_INCLUDE_DIRS}
    ${PC_ORCISIR_CONTROLLER_INCLUDE_DIRS}
    ${PC_PYTHON2_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
	${PC_XDECORE_LIBRARY_DIRS}
	${PC_OCL_LIBRARY_DIRS}
	${PC_OCL_DEPLOY_LIBRARY_DIRS}
	${PC_ORCISIR_CONTROLLER_LIBRARY_DIRS}
    ${PC_PYTHON2_LIBRARY_DIRS}
)

orocos_component(ISIRControllerThreadXDE
	ISIRControllerThreadXDE.cpp orcXdeModel.cpp ScenariosRomeo.cpp ScenariosICub.cpp ScenarioLibrary.cpp)

target_link_libraries(
	ISIRControllerThreadXDE
    ${Orocos-RTT_LIBRARIES}
	${PC_OCL_LIBRARIES}
	${PC_OCL_DEPLOY_LIBRARIES}
    ${PC_XDECORE_LIBRARIES}
	${PC_ORCISIR_CONTROLLER_LIBRARIES}
    ${PC_PYTHON2_LIBRARIES}
)

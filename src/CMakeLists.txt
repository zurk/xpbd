project(xpbd)
cmake_minimum_required(VERSION 3.19)
set(NAME xpbd)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE on)

file(GLOB ROOT_SOURCE *.cpp)
file(GLOB ROOT_INLINE *.inl)
file(GLOB ROOT_HEADER *.hpp)

file(GLOB_RECURSE CORE_SOURCE ./glm/core/*.cpp)
file(GLOB_RECURSE CORE_INLINE ./glm/core/*.inl)
file(GLOB_RECURSE CORE_HEADER ./glm/core/*.hpp)

file(GLOB_RECURSE GTC_SOURCE ./glm/gtc/*.cpp)
file(GLOB_RECURSE GTC_INLINE ./glm/gtc/*.inl)
file(GLOB_RECURSE GTC_HEADER ./glm/gtc/*.hpp)

file(GLOB_RECURSE GTX_SOURCE ./glm/gtx/*.cpp)
file(GLOB_RECURSE GTX_INLINE ./glm/gtx/*.inl)
file(GLOB_RECURSE GTX_HEADER ./glm/gtx/*.hpp)

file(GLOB_RECURSE VIRTREV_SOURCE ./glm/virtrev/*.cpp)
file(GLOB_RECURSE VIRTREV_INLINE ./glm/virtrev/*.inl)
file(GLOB_RECURSE VIRTREV_HEADER ./glm/virtrev/*.hpp)

source_group("Core Files" FILES ${CORE_SOURCE})
source_group("Core Files" FILES ${CORE_INLINE})
source_group("Core Files" FILES ${CORE_HEADER})
source_group("GTC Files" FILES ${GTC_SOURCE})
source_group("GTC Files" FILES ${GTC_INLINE})
source_group("GTC Files" FILES ${GTC_HEADER})
source_group("GTX Files" FILES ${GTX_SOURCE})
source_group("GTX Files" FILES ${GTX_INLINE})
source_group("GTX Files" FILES ${GTX_HEADER})
source_group("VIRTREV Files" FILES ${VIRTREV_SOURCE})
source_group("VIRTREV Files" FILES ${VIRTREV_INLINE})
source_group("VIRTREV Files" FILES ${VIRTREV_HEADER})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

add_executable(${NAME} 
	${ROOT_SOURCE}    ${ROOT_INLINE}    ${ROOT_HEADER} 
	${CORE_SOURCE}    ${CORE_INLINE}    ${CORE_HEADER}
	${GTC_SOURCE}     ${GTC_INLINE}     ${GTC_HEADER} 
	${GTX_SOURCE}     ${GTX_INLINE}     ${GTX_HEADER} 
	${VIRTREV_SOURCE} ${VIRTREV_INLINE} ${VIRTREV_HEADER})

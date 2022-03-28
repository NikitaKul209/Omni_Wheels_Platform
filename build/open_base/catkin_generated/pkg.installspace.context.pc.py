# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3;/usr/share/orocos_kdl/cmake/../../../include".split(';') if "${prefix}/include;/usr/include/eigen3;/usr/share/orocos_kdl/cmake/../../../include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;sensor_msgs;message_runtime;kdl_parser".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lopen_base;-lorocos-kdl".split(';') if "-lopen_base;-lorocos-kdl" != "" else []
PROJECT_NAME = "open_base"
PROJECT_SPACE_DIR = "/home/nikita/omni_ws/install"
PROJECT_VERSION = "0.0.0"

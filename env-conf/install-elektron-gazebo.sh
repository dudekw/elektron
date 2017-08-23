#!/bin/bash


export LANG=en_US.UTF-8

# useful functions
function usage {
  echo "usage: $0 <version> directory"
  echo "The <version> can be one of the following:"
  svn ls https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall.git/trunk/velma
  echo "latest"
  echo "latest_hw"
}

function printError {
    RED='\033[0;31m'
    NC='\033[0m' # No Color
    echo -e "${RED}$1${NC}"
}

build_dir="/$HOME/gazebo_ws"

distro="$ROS_DISTRO"

if [ "$distro" != "kinetic" ]; then
    printError "ERROR: ROS kinetic setup.bash have to be sourced!"
    exit 1
fi

# the list of packages that should be installed
installed=(
"python-wstool"
"ruby-dev"
"libprotobuf-dev"
"libprotoc-dev"
"protobuf-compiler"
"libtinyxml2-dev"
"libtar-dev"
"libtbb-dev"
"libfreeimage-dev"
"libignition-transport0-dev"
"omniorb"
"omniidl"
"libomniorb4-dev"
"libxerces-c-dev"
"doxygen"
"python-catkin-tools"
"libqtwebkit-dev"
"libgts-dev"
"libfcl-dev"
"libassimp-dev"
"libsimbody-dev"
"libogre-1.9-dev"
"ros-kinetic-diagnostic-updater"
)

# the list of packages that should be uninstalled
uninstalled=(
"ros-$distro-desktop-full"
"libdart"
"libsdformat"
"libignition-math2"
"gazebo"
)

error=false

# check the list of packages that should be installed
for item in ${installed[*]}
do
    aaa=`dpkg --get-selections | grep $item`
    if [ -z "$aaa" ]; then
        printError "ERROR: package $item is not installed. Please INSTALL it."
        error=true
    else
        arr=($aaa)
        name=${arr[0]}
        status=${arr[1]}
        if [ "$status" != "install" ]; then
            printError "ERROR: package $name is not installed. Please INSTALL it."
            error=true
        else
            echo "OK: package $name is installed"
        fi
    fi
done

# check the list of packages that should be uninstalled
for item in ${uninstalled[*]}
do
    aaa=`dpkg --get-selections | grep $item`
    if [ -z "$aaa" ]; then
        echo "OK: the package $item is not installed."
    else
        arr=($aaa)
        name=${arr[0]}
        status=${arr[1]}
        if [ "$status" != "install" ]; then
            echo "OK: the package $name is not installed."
        else
            printError "ERROR: package $name is installed. Please UNINSTALL it."
            error=true
        fi
    fi
done

if [ "$error" = true ]; then
    echo "Please install/uninstall the listed packages"
    echo "To install libccd please see http://askubuntu.com/questions/664101/dependency-in-ppa"
    exit 1
fi

if [ ! -d $build_dir ]; then
  mkdir $build_dir
fi

FRI_DIR=`pwd`

cd $build_dir

WORKSPACE_ROOT_DIR=`pwd`

if [ ! -e ".rosinstall" ]; then
  wstool init
fi

# test
#cp common_orocos.rosinstall       /tmp/common_orocos.rosinstall
#cp common_agent.rosinstall        /tmp/common_agent.rosinstall
#cp common_velma.rosinstall        /tmp/common_velma.rosinstall
#cp gazebo7_2_dart.rosinstall      /tmp/gazebo7_2_dart.rosinstall
#cp velma_hw.rosinstall            /tmp/velma_hw.rosinstall

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/RCPRG_rosinstall/master/gazebo7_2_dart.rosinstall      -O /tmp/gazebo7_2_dart.rosinstall
wstool merge /tmp/gazebo7_2_dart.rosinstall
wstool merge /tmp/velma.rosinstall
wstool update

# download package.xml for some packages
wget https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_dart-core.xml -O underlay_isolated/src/gazebo/dart/package.xml
wget https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml  -O underlay_isolated/src/gazebo/sdformat/package.xml
wget https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml    -O underlay_isolated/src/gazebo/gazebo/package.xml
wget https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_ign-math.xml  -O underlay_isolated/src/gazebo/ign-math/package.xml

# patch for gazebo -> set fsaa to 0
#
wget https://raw.githubusercontent.com/dudekw/gazebo-fsaa-patch/master/Camera.cc            -O $WORKSPACE_ROOT_DIR/underlay_isolated/src/gazebo/gazebo/gazebo/rendering/Camera.cc
#

#
# underlay_isolated
#
cd $WORKSPACE_ROOT_DIR/underlay_isolated
catkin config --cmake-args -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB -DCMAKE_BUILD_TYPE=Release -DBUILD_CORE_ONLY=ON   -DBUILD_SHARED_LIBS=ON   -DUSE_DOUBLE_PRECISION=ON -DENABLE_MQUEUE=ON -DBUILD_HELLOWORLD=OFF -DENABLE_TESTS_COMPILATION=False -DENABLE_SCREEN_TESTS=False
catkin build
if [ $? -eq 0 ]; then
    echo "underlay_isolated build OK"
else
    printError "underlay_isolated build FAILED"
    exit 1
fi

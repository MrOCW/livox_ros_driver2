#!/usr/bin/env bash
set -euo pipefail

readonly VERSION_ROS2="ROS2"
readonly VERSION_JAZZY="jazzy"
readonly VERSION_LYRICAL="lyrical"

pushd "$(pwd)" > /dev/null
cd "$(dirname "$0")"
echo "Working Path: $(pwd)"

requested_distro="${1:-${ROS_DISTRO:-}}"
requested_distro="${requested_distro,,}"

case "${requested_distro}" in
    "${VERSION_JAZZY}"|"${VERSION_LYRICAL}")
        ROS_DISTRO="${requested_distro}"
        ;;
    "")
        echo "Usage: $0 {jazzy|lyrical}"
        exit 1
        ;;
    *)
        echo "Unsupported ROS 2 distro: ${requested_distro}"
        echo "Supported distros: jazzy, lyrical"
        exit 1
        ;;
esac

echo "ROS version is: ${VERSION_ROS2}"
echo "ROS distro is: ${ROS_DISTRO}"

workspace_dir="$(cd ../../.. && pwd)"
previous_ros=""
if [ -f "${workspace_dir}/install/setup.bash" ]; then
    previous_ros="$(sed -n 's|.*/opt/ros/\([^"]*\)".*|\1|p' "${workspace_dir}/install/setup.bash" | head -n 1)"
fi

echo "PREVIOUS ROS DISTRO: ${previous_ros}"
if [ "${ROS_DISTRO}" != "${previous_ros}" ]; then
    echo "clear build folder"
    rm -rf "${workspace_dir}/build/"
    rm -rf "${workspace_dir}/install/"
    rm -rf "${workspace_dir}/log/"
else
    echo "build folder already here"
fi

cd "${workspace_dir}"
colcon build --symlink-install --cmake-args \
    -DROS_EDITION=${VERSION_ROS2} \
    -DDISTRO_ROS=${ROS_DISTRO} \
    -DCMAKE_BUILD_TYPE=Release \
    -Wno-dev

popd > /dev/null

#!/bin/bash
# Phase 2 Verification Script
# Tests all requirements for the executable IMU transformer node

set -e  # Exit on any error

WORKSPACE="/home/ggsya/ros2_ws"
PACKAGE="rtabmap_examples"
NODE="imu_axis_transformer.py"

echo "=========================================="
echo "Phase 2 Verification Script"
echo "IMU Axis Transformer Node"
echo "=========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check functions
check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    exit 1
}

check_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

# 1. Check source file exists and is executable
echo "1. Checking source file..."
if [ -f "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/scripts/${NODE}" ]; then
    check_pass "Source file exists"
else
    check_fail "Source file not found"
fi

if [ -x "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/scripts/${NODE}" ]; then
    check_pass "Source file is executable"
else
    check_fail "Source file is not executable"
fi
echo ""

# 2. Check shebang line
echo "2. Checking shebang line..."
FIRST_LINE=$(head -n 1 "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/scripts/${NODE}")
if [ "$FIRST_LINE" = "#!/usr/bin/env python3" ]; then
    check_pass "Shebang line is correct"
else
    check_fail "Shebang line is incorrect: $FIRST_LINE"
fi
echo ""

# 3. Check for main entry point
echo "3. Checking main entry point..."
if grep -q "def main(args=None):" "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/scripts/${NODE}"; then
    check_pass "main() function exists"
else
    check_fail "main() function not found"
fi

if grep -q "if __name__ == '__main__':" "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/scripts/${NODE}"; then
    check_pass "__main__ block exists"
else
    check_fail "__main__ block not found"
fi

if grep -q "rclpy.init" "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/scripts/${NODE}"; then
    check_pass "rclpy.init() call exists"
else
    check_fail "rclpy.init() not found"
fi

if grep -q "rclpy.spin" "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/scripts/${NODE}"; then
    check_pass "rclpy.spin() call exists"
else
    check_fail "rclpy.spin() not found"
fi
echo ""

# 4. Check CMakeLists.txt install directive
echo "4. Checking CMakeLists.txt..."
CMAKE_FILE="${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/CMakeLists.txt"
if grep -q "install(PROGRAMS" "$CMAKE_FILE"; then
    check_pass "install(PROGRAMS directive exists"
else
    check_fail "install(PROGRAMS directive not found"
fi

if grep -q "scripts/${NODE}" "$CMAKE_FILE"; then
    check_pass "Script included in install directive"
else
    check_fail "Script not in install directive"
fi

if grep -q "DESTINATION lib/\${PROJECT_NAME}" "$CMAKE_FILE"; then
    check_pass "Install destination is correct"
else
    check_fail "Install destination is incorrect"
fi
echo ""

# 5. Check package.xml dependencies
echo "5. Checking package.xml dependencies..."
PACKAGE_XML="${WORKSPACE}/src/rtabmap_ros/${PACKAGE}/package.xml"
if grep -q "<exec_depend>rclpy</exec_depend>" "$PACKAGE_XML"; then
    check_pass "rclpy dependency exists"
else
    check_fail "rclpy dependency missing"
fi

if grep -q "<exec_depend>sensor_msgs</exec_depend>" "$PACKAGE_XML"; then
    check_pass "sensor_msgs dependency exists"
else
    check_fail "sensor_msgs dependency missing"
fi

if grep -q "<exec_depend>python3-numpy</exec_depend>" "$PACKAGE_XML"; then
    check_pass "python3-numpy dependency exists"
else
    check_fail "python3-numpy dependency missing"
fi
echo ""

# 6. Build the package
echo "6. Building package..."
cd "$WORKSPACE"
if colcon build --packages-select "$PACKAGE" --symlink-install 2>&1 | grep -q "Finished <<<"; then
    check_pass "Package built successfully"
else
    check_fail "Package build failed"
fi
echo ""

# 7. Source workspace
echo "7. Sourcing workspace..."
source "${WORKSPACE}/install/setup.bash"
check_pass "Workspace sourced"
echo ""

# 8. Check node is discoverable
echo "8. Checking node is discoverable..."
if ros2 pkg executables "$PACKAGE" | grep -q "$NODE"; then
    check_pass "Node is discoverable via ros2 pkg executables"
else
    check_fail "Node not found via ros2 pkg executables"
fi
echo ""

# 9. Check installation
echo "9. Checking installation..."
INSTALL_PATH="${WORKSPACE}/install/${PACKAGE}/lib/${PACKAGE}/${NODE}"
if [ -f "$INSTALL_PATH" ]; then
    check_pass "Node installed to correct location"
else
    check_fail "Node not found at install location"
fi

if [ -x "$INSTALL_PATH" ]; then
    check_pass "Installed node is executable"
else
    check_fail "Installed node is not executable"
fi
echo ""

# 10. Run tests
echo "10. Running unit tests..."
cd "${WORKSPACE}/src/rtabmap_ros/${PACKAGE}"
if python3 -m pytest test/test_imu_axis_transformer.py -v 2>&1 | grep -q "11 passed"; then
    check_pass "All 11 tests passed"
else
    check_fail "Some tests failed"
fi
echo ""

# 11. Test node can be launched (quick startup test)
echo "11. Testing node can be launched..."
cd "$WORKSPACE"
source "${WORKSPACE}/install/setup.bash"
if timeout 2 ros2 run "$PACKAGE" "$NODE" 2>&1 | grep -q "IMU Axis Transformer started"; then
    check_pass "Node launches successfully"
else
    check_fail "Node failed to launch"
fi
echo ""

# Final summary
echo "=========================================="
echo -e "${GREEN}Phase 2 Verification COMPLETE${NC}"
echo "=========================================="
echo ""
echo "All requirements met:"
echo "  ✓ Script has proper shebang and main entry point"
echo "  ✓ CMakeLists.txt has install directive"
echo "  ✓ package.xml has all dependencies"
echo "  ✓ Package builds successfully"
echo "  ✓ Node is discoverable and installable"
echo "  ✓ All 11 unit tests pass"
echo "  ✓ Node can be launched"
echo ""
echo "The IMU axis transformer node is ready for use!"
echo ""
echo "To run the node:"
echo "  ros2 run ${PACKAGE} ${NODE}"
echo ""

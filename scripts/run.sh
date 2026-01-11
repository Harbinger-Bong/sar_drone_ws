#!/bin/bash

# SAR Drone Launch Script
# Launch the complete SAR drone simulation in separate terminals

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get workspace directory
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo -e "${BLUE}🚁 Launching SAR Drone Simulation...${NC}"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo -e "${RED}❌ Workspace not built. Running build first...${NC}"
    ./scripts/build.sh
fi

# Source the environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Function to launch in new terminal
launch_in_terminal() {
    local title="$1"
    local command="$2"
    echo -e "${BLUE}🖥️  Launching $title...${NC}"
    
    # Try different terminal emulators
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "cd '$WORKSPACE_DIR' && source /opt/ros/foxy/setup.bash && source install/setup.bash && $command; exec bash"
    elif command -v xterm &> /dev/null; then
        xterm -title "$title" -e "cd '$WORKSPACE_DIR' && source /opt/ros/foxy/setup.bash && source install/setup.bash && $command; exec bash" &
    elif command -v konsole &> /dev/null; then
        konsole --title "$title" -e bash -c "cd '$WORKSPACE_DIR' && source /opt/ros/foxy/setup.bash && source install/setup.bash && $command; exec bash" &
    else
        echo -e "${YELLOW}⚠️  No supported terminal found. Please run manually:${NC}"
        echo "cd $WORKSPACE_DIR"
        echo "source /opt/ros/foxy/setup.bash && source install/setup.bash"
        echo "$command"
        return 1
    fi
}

echo -e "${YELLOW}📋 Launching components in separate terminals...${NC}"
echo ""

# Launch Gazebo simulation
launch_in_terminal "SAR Drone - Gazebo" "ros2 launch sar_drone_description sar_world.launch.py"
sleep 3

# Launch Navigation stack
launch_in_terminal "SAR Drone - Navigation" "ros2 launch sar_drone_description sar_nav2.launch.py"
sleep 3

# Launch RViz
launch_in_terminal "SAR Drone - RViz" "ros2 launch nav2_bringup rviz_launch.py"

echo ""
echo -e "${GREEN}🎉 SAR Drone simulation launched!${NC}"
echo ""
echo -e "${YELLOW}📝 What's running:${NC}"
echo "🔹 Terminal 1: Gazebo simulation with SAR world"
echo "🔹 Terminal 2: Nav2 navigation stack with SLAM"
echo "🔹 Terminal 3: RViz visualization"
echo ""
echo -e "${BLUE}🎮 Usage:${NC}"
echo "• Set initial pose in RViz using '2D Pose Estimate'"
echo "• Set navigation goals using '2D Nav Goal'"
echo "• Monitor topics: ros2 topic list"
echo "• View camera: ros2 run rqt_image_view rqt_image_view"
echo ""
echo -e "${GREEN}Happy flying! 🚁${NC}"

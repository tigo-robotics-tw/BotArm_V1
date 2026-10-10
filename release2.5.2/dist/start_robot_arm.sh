#!/bin/bash
#
# Robot Arm Control - Unified Application Launcher
# Copyright (C) 2026 BigHead theBit <student15.tigo.robotics@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# Robot Arm Control - Unified Application Launcher
# Launches single Python application with GUI, WebSocket server, and motor control

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Robot Arm Control - Startup${NC}"
echo -e "${GREEN}================================${NC}"
echo ""

# Check if Python3 is installed
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Error: python3 is not installed${NC}"
    exit 1
fi

# Check if required Python packages are installed
echo -e "${YELLOW}Checking dependencies...${NC}"
python3 -c "import websockets" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: websockets package not installed${NC}"
    echo -e "${YELLOW}Install with: pip3 install websockets pyserial${NC}"
    exit 1
fi

python3 -c "import serial" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: pyserial package not installed${NC}"
    echo -e "${YELLOW}Install with: pip3 install websockets pyserial${NC}"
    exit 1
fi

python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}Error: tkinter not installed${NC}"
    echo -e "${YELLOW}Install with: sudo apt-get install python3-tk${NC}"
    exit 1
fi

echo -e "${GREEN}✓ All dependencies found${NC}"
echo ""

# Check if RobotArmControlApp.py exists
if [ ! -f "RobotArmControlApp.py" ]; then
    echo -e "${RED}Error: RobotArmControlApp.py not found${NC}"
    exit 1
fi

# Check if HTML file exists
HTML_FILE="$SCRIPT_DIR/RobotArmControl.html"
if [ ! -f "$HTML_FILE" ]; then
    echo -e "${RED}Error: RobotArmControl.html not found${NC}"
    exit 1
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Starting Unified Robot Arm Control${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}This will start a unified application with:${NC}"
echo -e "  • GUI for configuration and motor control"
echo -e "  • WebSocket server (ws://localhost:8765)"
echo -e "  • Single serial port connection to motors"
echo ""
echo -e "${BLUE}Features:${NC}"
echo -e "  • Tab 1: Connection & Motor Control"
echo -e "  • Tab 2: Robot Configuration"
echo -e "  • Tab 3: WebSocket Server Status"
echo ""
echo -e "${YELLOW}Starting application...${NC}"
echo ""

# Run the unified application
python3 RobotArmControlApp.py

echo ""
echo -e "${GREEN}Application closed.${NC}"

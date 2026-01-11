#!/bin/bash

# ROS 2 Network Configuration Verification Script
# This script verifies that ROS 2 is properly configured for network communication

echo "🔍 Verifying ROS 2 Network Configuration..."
echo ""

# Check ROS_DOMAIN_ID
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "❌ ROS_DOMAIN_ID is not set"
    exit 1
else
    echo "✅ ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
fi

# Check ROS_LOCALHOST_ONLY
if [ -z "$ROS_LOCALHOST_ONLY" ]; then
    echo "⚠️  ROS_LOCALHOST_ONLY is not set (defaults to 1 - localhost only)"
    exit 1
elif [ "$ROS_LOCALHOST_ONLY" = "0" ]; then
    echo "✅ ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY (network enabled)"
else
    echo "❌ ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY (should be 0 for network communication)"
    exit 1
fi

# Check RMW_IMPLEMENTATION
if [ -z "$RMW_IMPLEMENTATION" ]; then
    echo "⚠️  RMW_IMPLEMENTATION is not set"
else
    echo "✅ RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
fi

# Check ROS 2 installation
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS 2 is not installed or not in PATH"
    exit 1
else
    echo "✅ ROS 2 command available"
fi

# Check ROS 2 topics
echo ""
echo "📡 Testing ROS 2 topic list..."
if ros2 topic list &> /dev/null; then
    echo "✅ ROS 2 is working correctly"
    echo ""
    echo "Available topics:"
    ros2 topic list
else
    echo "❌ Failed to list ROS 2 topics"
    exit 1
fi

echo ""
echo "🎉 All checks passed! ROS 2 is configured for network communication."
echo ""
echo "📝 Next steps:"
echo "   1. Ensure your ground station (Humble) has the same configuration"
echo "   2. Both machines must use ROS_DOMAIN_ID=7"
echo "   3. Both machines must have ROS_LOCALHOST_ONLY=0"
echo "   4. Both machines should use RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo "   5. Start your nodes (camera, LiDAR, mavros, Gazebo, etc.)"

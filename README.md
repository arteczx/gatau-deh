# 🚀 JAXA Kibo-RPC Space Robot Programming Challenge 🛸

## 🌟 Project Overview
This repository contains code for the JAXA Kibo-RPC competition, where participants program Astrobee robots to perform autonomous missions on the International Space Station. 🌎

## ✨ Key Features
- 🎯 Optimized path planning with safety checks for Keep In Zones (KIZ) and Keep Out Zones (KOZ)
- ⚡ Multi-threaded execution for concurrent movement and image processing
- 👁️ AR marker detection using OpenCV and ArUco markers
- 🛡️ Robust error handling and retry mechanisms
- 💾 Efficient caching and resource management

## 🔧 Technical Details

### 🧭 Navigation & Safety
- 🚧 Implements boundary checking for KIZ/KOZ zones
- ⚖️ Path safety validation with acceleration and thrust limits
- 🎯 Safe waypoint system & A* (A-Star) Pathfinding for obstacle avoidance
- 🔄 Quaternion-based orientation control

### 📸 Computer Vision
- 🎯 AR marker detection using ArUco library
- 🖼️ Optimized image processing with adaptive thresholding
- 💽 Image caching for improved performance
- 📷 Camera calibration handling

### ⚡ Concurrency
- 🔄 Multi-threaded execution using ExecutorService
- ⚡ Parallel movement and image processing
- 🔒 Thread-safe data structures
- 🧹 Proper resource cleanup

### ⚠️ Error Handling
- 🔄 Retry mechanisms for movement commands
- ⏱️ Timeout handling for operations
- 🛟 Graceful failure recovery
- 📝 Comprehensive error logging

## 📁 Project Structure
- 📄 `YourService.java`: Main service class implementing mission logic
- Key components:
  - 🎮 Mission execution methods
  - 🧭 Navigation helpers
  - 👁️ Computer vision processing
  - ✅ Safety validation functions

## 📦 Dependencies
- 🤖 Astrobee Robot Software
- 📸 OpenCV (with ArUco module)
- 🛰️ JAXA Kibo-RPC API

## 👥 Authors
Team FlyOverSpace 🌠

## 📜 License
This project is developed for the JAXA Kibo-RPC competition. 🚀

# Video-Stabilization

This is a comprehensive VHDL video stabilization system for processing HDMI video feeds. Here's what the system includes:
Main Components:

HDMI Receiver - Captures and processes incoming HDMI video signals
Motion Detector - Analyzes consecutive frames to detect camera shake/movement
Frame Buffers - Stores current and previous frames for comparison
Stabilization Processor - Calculates compensation offsets using filtering algorithms
Video Output - Applies stabilization and outputs corrected HDMI video

Key Features:

Real-time Processing: Processes video at pixel clock rates
Motion Detection: Uses block-matching algorithm to detect inter-frame motion
Digital Stabilization: Applies low-pass filtering to smooth out unwanted motion
Configurable Parameters: Adjustable motion threshold and stabilization enable
Memory Management: Efficient frame buffering with DDR3 interface
Status Monitoring: LED indicators for system status

How It Works:

Capture: HDMI receiver captures incoming video frames
Buffer: Current and previous frames are stored in memory
Analyze: Motion detector compares frames to calculate motion vectors
Filter: Stabilization processor applies smoothing algorithms
Compensate: Output stage applies calculated offsets to stabilize the image
Output: Corrected video is sent to HDMI output

Implementation Notes:

The system assumes 1280x720 resolution but can be adapted for other formats
Uses simplified RGB to grayscale conversion for motion detection
Implements a low-pass filter for smooth stabilization
Requires external DDR3 memory for frame storage
Can be synthesized for most FPGA platforms with HDMI capabilities

To use this system, you'll need to:

Implement the missing components (hdmi_receiver, frame_buffer, video_output)
Add appropriate clock management (PLLs/DCMs)
Configure the DDR3 memory controller
Adjust parameters for your specific video format
Add any necessary HDMI PHY interfaces

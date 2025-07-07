# Video-Stabilization

Alternate GitHub repos to look into:

https://github.com/fcayci/vhdl-hdmi-out

Other supporting links:

https://www.fpga4fun.com/HDMI.html
https://hackaday.com/2020/06/02/an-open-source-hdmi-implementation-for-fpgas/
https://forum.digilent.com/topic/20043-add-vhdl-module-to-hdmi-demo-block-diagram/

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

# Main HDMI Receiver (hdmi_receiver)
Key Features:

Automatic Format Detection: Measures timing parameters and detects video resolution
State Machine Control: Robust state management for sync detection and video processing
Clock Domain Synchronization: Proper handling of HDMI clock domain
Lock Detection: Monitors signal stability and indicates when video is locked
Frame Rate Measurement: Calculates and reports video frame rate
Debug Support: Provides state information for troubleshooting

Input/Output Interface:

HDMI Inputs: Clock, 24-bit RGB data, DE, HSync, VSync
Video Outputs: Pixel clock, data, validity signals, frame/line start pulses
Status Outputs: Lock indication, resolution, frame rate, debug state

State Machine:

IDLE: Wait for sync signals
SYNC_DETECT: Look for consistent sync patterns
TIMING_MEASURE: Measure active video timing
VIDEO_ACTIVE: Normal video processing and output
FRAME_SYNC: Resynchronization when needed

Enhanced Clock Recovery (hdmi_clock_recovery)
Features:

TMDS Decoding: Handles differential HDMI signals
Clock Recovery: Extracts pixel clock from HDMI stream
Multi-Channel Decoding: Separate processing for R, G, B channels
Control Signal Extraction: Recovers sync and DE signals

Key Components:

Differential input buffers
Clock recovery PLL
TMDS deserializers
Channel decoders

Implementation Notes:
Platform-Specific Elements:

Clock Buffers: Use IBUFDS (Xilinx) or equivalent for differential inputs
PLLs: Use MMCM/PLL primitives for clock generation
Deserializers: Use ISERDES or similar for high-speed data recovery

Timing Parameters:

Default configured for 1280x720@60Hz
Automatically adapts to detected video format
Supports common HDMI/DVI video standards

Usage Example:
vhdlhdmi_rx_inst: hdmi_receiver
    port map (
        clk => clk_100mhz,
        rst => reset,
        hdmi_clk => hdmi_pixel_clk,
        hdmi_data => hdmi_rgb_data,
        hdmi_de => hdmi_data_enable,
        hdmi_hsync => hdmi_h_sync,
        hdmi_vsync => hdmi_v_sync,
        pixel_clk => video_clk,
        pixel_data => video_data,
        pixel_valid => video_valid,
        frame_start => frame_sync,
        line_start => line_sync,
        video_locked => video_lock_status
    );
Integration with Video Stabilizer:
This HDMI receiver integrates seamlessly with the video stabilization system I created earlier. The pixel_clk, pixel_data, pixel_valid, frame_start, and line_start outputs connect directly to the stabilization system inputs.

Frame Buffer Module (frame_buffer)
Key Features:

Dual-Port Memory: Simultaneous read/write operations
Configurable Size: Parameterized address and data width
BRAM Inference: Optimized for FPGA block RAM resources
Status Monitoring: Buffer full/empty indicators
Cross-Clock Domain: Separate read/write clock domains

Usage:

Stores video frames for motion analysis and stabilization
Supports up to 1M pixels (suitable for 1280x720 and higher)
Automatically infers BRAM for efficient memory usage

Ping-Pong Frame Buffer (ping_pong_frame_buffer)
Advanced Features:

Double Buffering: Automatic ping-pong operation
Frame Synchronization: Seamless frame switching
Continuous Operation: No frame drops during buffer switching
Automatic Address Management: Built-in address generation

Benefits:

Enables real-time processing without frame drops
Smooth transitions between frames
Ideal for video stabilization applications

Video Output Module (video_output)
Core Features:

Video Timing Generation: Standard HDMI/DVI timing signals
Stabilization Integration: Applies X/Y offset corrections
Border Handling: Fills areas outside stabilized image
Configurable Timing: Supports various video formats

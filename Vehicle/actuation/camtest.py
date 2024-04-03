import cv2
import numpy as np
import pyrealsense2 as rs
import os

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

image_path = '/home/jetson/self_driving/vision/RGB'
depth_path =  '/home/jetson/self_driving/vision/Depth'

# Start streaming
pipeline.start(config)

try:
    frame = 0
    while True:
        # Wait for a coherent pair of frames: depth and color\
        
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        rgb_image = np.asanyarray(color_frame.get_data())

        # Apply colormap to the depth image
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.2), cv2.COLORMAP_JET)

        # Display the depth colormap
        cv2.imshow('RealSense RGB', rgb_image)
        cv2.imshow('RealSense Depth', depth_colormap)

        os.chdir(image_path) 
        cv2.imwrite(f'{frame}.png',rgb_image)
        os.chdir(depth_path) 
        cv2.imwrite(f'{frame}.png',depth_image)

        frame+=1

        # Break the loop when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

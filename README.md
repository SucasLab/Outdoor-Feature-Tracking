# Outdoor-Feature-Tracking
This is the outdoor feature tracking module in paper "Model Predictive Control with Residual Learning and Real-time Disturbance Rejection: Design and Experimentation"

We provide a sample test video and a processed output video for reference. You can also replace it with your own video for testing.

🚀 Quick Start

Modify the video path in objectTracking_withAngle_withKCF.py:

cap = cv2.VideoCapture("path/to/your/video.mp4")  # Replace "path/to/your/video.mp4" with your video file path

Run the Python script and follow the prompts to input the required information:

python objectTracking_withAngle_withKCF.py

​​    2.1​​ Enter the X-coordinate (in world coordinates) of the center point of the top-left label (press Enter to confirm).

​    ​2.2​​ Enter the Y-coordinate (in world coordinates) of the center point of the top-left label (press Enter to confirm).

​    ​2.3​​ Enter the distance (in meters) between the center points of the top-left and top-right labels (press Enter to confirm).

    ​​2.4​​ Enter the number of labels used for relative positioning on the ground (press Enter to confirm).

Use the pop-up window to annotate the label areas and target area (press Enter to confirm each selection).

⚠️ ​​Note​​: When annotating labels, please follow the order:

        0: left-top
        1: right-top
        2: right-bottom
        3: left-bottom

This order is necessary for subsequent field-of-view angle correction and relative position calculation.

The target's pose information is published via ROS topics by default. You can subscribe to or record these topics for further use.

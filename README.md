# Outdoor-Feature-Tracking
This is the outdoor feature tracking module in paper "Model Predictive Control with Residual Learning and Real-time Disturbance Rejection: Design and Experimentation"

🚀 Quick Start

We provide a sample video and its processed output for reference. You can also use your own video for testing.
📋 Prerequisites

Ensure you have OpenCV and other required dependencies installed.


🛠️ Usage

​​Update Video Path​​:

Modify the video path in objectTracking_withAngle_withKCF.py:

cap = cv2.VideoCapture("path/to/your/video.mp4")  # Replace with your video path

​​Run the Script​​:

Execute the Python script and follow the prompts:

python objectTracking_withAngle_withKCF.py

​​Input Parameters​​:

Follow the interactive prompts to enter the required information:

​​X-coordinate​​ of the top-left label's center in the world coordinate system (press Enter to confirm).

​​Y-coordinate​​ of the top-left label's center in the world coordinate system (press Enter to confirm).

​​Distance (in meters)​​ between the top-left and top-right label centers (press Enter to confirm).

    ​​Number of Labels​​ used for ground relative positioning (press Enter to confirm).

​​Annotation​​:

An image window will pop up for annotation.

​​Label Annotation​​: Label the regions in the following order:

0: left-top→ 1: right-top→ 2: right-bottom→ 3: left-bottom.

This order is crucial for field-of-view correction and relative position calculation.

    ​​Target Annotation​​: Mark the target region (press Enter to complete).

    ​​Output​​:

    The target's pose information is published via ROS topics by default. You can subscribe to or record these topics for further use.

💡 Notes

Ensure accurate label annotation order to avoid errors in angle correction and positioning.

    The script assumes specific label configurations for optimal performance.

For detailed information about project structure, technical stack, or contribution guidelines, please refer to other sections of this README.

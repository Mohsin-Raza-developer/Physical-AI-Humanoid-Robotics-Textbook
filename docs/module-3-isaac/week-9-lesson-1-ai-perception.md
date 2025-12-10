---
title: Perception Pipelines
sidebar_label: "Lesson 1: AI Perception"
sidebar_position: 91
description: Building perception pipelines with Isaac
tags: [isaac, perception, week-9]
---

# Perception Pipelines

Perception is a core component of robotics that enables robots to understand their environment.

## Computer Vision

- Object detection and recognition
- Semantic segmentation
- Depth estimation

## Sensor Fusion

- Multi-sensor integration
- Kalman filtering
- Data association

## Example Code

Here is an example of computer vision code using Isaac:

```python
# Isaac ROS example for object detection
import numpy as np
import cv2
from isaac_ros_interfaces.msg import Detection2DArray

def process_image(image):
    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform edge detection
    edges = cv2.Canny(blurred, 50, 150)
    
    return edges
```

```cpp
// Example of Isaac SIM perception pipeline
#include <torch/torch.h>
#include <opencv2/opencv.hpp>

class PerceptionPipeline {
public:
    torch::Tensor detectObjects(cv::Mat& image) {
        // Convert OpenCV Mat to PyTorch tensor
        auto tensor = torch::from_blob(
            image.data, 
            {image.rows, image.cols, image.channels()}, 
            torch::kUInt8
        );
        
        // Normalize the input
        tensor = tensor.permute({2, 0, 1}).to(torch::kFloat) / 255.0;
        
        // Run the detection model
        torch::NoGradGuard no_grad;
        auto detections = detection_model_.forward({tensor});
        
        return detections.toTensor();
    }

private:
    torch::jit::script::Module detection_model_;
};
```
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Requesting to move robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the service drive_robot
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service drive_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image& img)
{

    bool found_ball = false;

    constexpr float kSpeedForward = 0.5F;
    constexpr float kSpeedRotation = 0.5F;

    constexpr uint32_t kNrChannels = 3U;

    constexpr uint8_t kWhitePixel = 255U;

    const uint32_t kLeftBoundary = img.width / 3U;
    const uint32_t kRightBoundary = (2U * img.height) / 3U;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (uint32_t i = 0U; i < img.height; ++i)
    {
        for (uint32_t j = 0U; j < img.width; ++j)
        {
            const uint32_t idx_base = (i * img.width * kNrChannels) + j * kNrChannels;
            const uint8_t pixel_r = img.data[idx_base + 0U];
            const uint8_t pixel_g = img.data[idx_base + 1U];
            const uint8_t pixel_b = img.data[idx_base + 2U];

            if ((pixel_r == kWhitePixel) && (pixel_g == kWhitePixel) && (pixel_b == kWhitePixel))
            {
                if (j < kLeftBoundary)
                {
                    // Left part of the image -> turn left
                    drive_robot(0.F, kSpeedRotation);
                }
                else if (j > kRightBoundary)
                {
                    // Right part of the image -> turn right
                    drive_robot(0.F, -kSpeedRotation);
                }
                else
                {
                    // Center part of the image -> move forward
                    drive_robot(kSpeedForward, 0.F);
                }
                return;
            }
        }
    }

    // Stop the robot if ball not found
    drive_robot(0.F, 0.F);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 2, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

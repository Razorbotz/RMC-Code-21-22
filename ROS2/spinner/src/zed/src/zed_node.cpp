#include <rclcpp/rclcpp.hpp>

#include<sl/Camera.hpp>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/msg/image.hpp>


/** @file
 * @brief Node handling ZED camera
 * 
 * This node does not subscribe to any nodes.
 * 
 * The topics that are being published are as follows:
 * TODO: create publisher for images
 * 
 * \see communication_node.cpp
 * */

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("zed");
    auto zedPublisher = nodeHandle->create_publisher<sensor_msgs::msg::Image>("zed_image",1);

    RCLCPP_INFO(nodeHandle->get_logger(), "Starting ZED node");

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::VGA;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init_params.sensors_required = false;

    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
	    std::cout << "Error, unable to open ZED camera: " << err << "\n";
        zed.close();
        return 1; // Quit if an error occurred
    }
    auto cameraInfo = zed.getCameraInformation();
    sl::Resolution image_size = cameraInfo.camera_resolution;
    sl::Mat image_zed(image_size, sl::MAT_TYPE::U8_C4);
    cv::Mat image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));
    cv::Mat image_ocv_rgb;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;

    rclcpp::Rate rate(10);
    while(rclcpp::ok()){
        if(zed.grab() == sl::ERROR_CODE::SUCCESS){
            RCLCPP_INFO(nodeHandle->get_logger(), "Captured image");
            zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
            //publish image
//            auto stamp = now();
            sensor_msgs::msg::Image::UniquePtr outImage(new sensor_msgs::msg::Image());

//            outImage->header.stamp = stamp;
            //Fix this
//            outImage->header.frame_id = ?;
            outImage->height = image_ocv.rows;
            outImage->width = image_ocv.cols;
            outImage->encoding = "rgb";
            outImage->is_bigendian = false;
            outImage->step = static_cast<sensor_msgs::msg::Image::_step_type>(image_ocv.step);
            outImage->data.assign(image_ocv.datastart, image_ocv.dataend);
            zedPublisher->publish(std::move(outImage));
        }
        rate.sleep();
    }
    zed.close();
    return 0;
}

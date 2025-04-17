#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>
#include <zmq.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "nlohmann/json.hpp"

void ProcessImage(ORB_SLAM3::System& SLAM, const cv::Mat& im, const double& tframe, 
                  zmq::socket_t& pose_publisher, zmq::socket_t& features_publisher,
                  const std::vector<uchar>& jpeg_data)
{
    cv::Mat im_copy = im.clone();
    
    Sophus::SE3f pose = SLAM.TrackMonocular(im_copy, tframe);

    if(SLAM.GetTrackingState() == ORB_SLAM3::Tracking::OK)
    {
        Eigen::Matrix4f mat = pose.matrix();
        float tx = mat(0,3);
        float ty = mat(1,3);
        float tz = mat(2,3);
        
        std::stringstream ss;
        ss << tframe << "," << tx << "," << ty << "," << tz;
        std::string coords_str = ss.str();
        
        zmq::message_t pose_message(coords_str.size());
        memcpy(pose_message.data(), coords_str.data(), coords_str.size());
        pose_publisher.send(pose_message, zmq::send_flags::none);

        std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
        std::vector<cv::KeyPoint> key_points = SLAM.GetTrackedKeyPointsUn();
        
        nlohmann::json root;
        root["timestamp"] = tframe;
        root["image_size"]["width"] = im.cols;
        root["image_size"]["height"] = im.rows;

        std::stringstream jpeg_base64;
        for (size_t i = 0; i < jpeg_data.size(); ++i) {
            jpeg_base64 << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(jpeg_data[i]);
        }
        root["image_data"] = jpeg_base64.str();

        nlohmann::json points = nlohmann::json::array();
        
        for (size_t i = 0; i < key_points.size(); i++) {
            if (i < map_points.size() && map_points[i] != NULL) {
                nlohmann::json point;

                point["image_x"] = key_points[i].pt.x;
                point["image_y"] = key_points[i].pt.y;
                point["size"] = key_points[i].size;
                point["angle"] = key_points[i].angle;
                point["response"] = key_points[i].response;
                point["octave"] = key_points[i].octave;

                Eigen::Vector3f pos = map_points[i]->GetWorldPos();
                point["world_x"] = pos(0);
                point["world_y"] = pos(1);
                point["world_z"] = pos(2);
                
                points.push_back(point);
            }
        }
        
        root["points"] = points;
        root["num_points"] = points.size();

        std::string json_data = root.dump();

        zmq::message_t features_message(json_data.size());
        memcpy(features_message.data(), json_data.data(), json_data.size());
        features_publisher.send(features_message, zmq::send_flags::none);
    }
    else
    {
        std::cout << "Tracking lost!" << std::endl;
    }
}

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_vocabulary> <path_to_settings> "
                  << "<zmq_image_address> <zmq_pose_address> <zmq_features_address>" << std::endl;
        return 1;
    }

    const char* vocabulary_path = argv[1];
    const char* settings_path = argv[2];
    const char* zmq_image_address = argv[3];
    const char* zmq_pose_address = argv[4];
    const char* zmq_features_address = argv[5];

    ORB_SLAM3::System SLAM(vocabulary_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);

    zmq::context_t context(1);

    zmq::socket_t image_subscriber(context, ZMQ_SUB);
    try {
        image_subscriber.connect(zmq_image_address);
        image_subscriber.set(zmq::sockopt::subscribe, "");
        std::cout << "Connected to image stream at " << zmq_image_address << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect to ZMQ image server: " << e.what() << std::endl;
        return 1;
    }

    zmq::socket_t pose_publisher(context, ZMQ_PUB);
    try {
        pose_publisher.connect(zmq_pose_address);
        std::cout << "Publishing pose data to " << zmq_pose_address << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect pose publisher: " << e.what() << std::endl;
        return 1;
    }

    zmq::socket_t features_publisher(context, ZMQ_PUB);
    try {
        features_publisher.connect(zmq_features_address);
        std::cout << "Publishing features data to " << zmq_features_address << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect features publisher: " << e.what() << std::endl;
        return 1;
    }

    while (true)
    {
        zmq::message_t message;
        zmq::recv_result_t result = image_subscriber.recv(message, zmq::recv_flags::none);
        if (!result) {
            std::cerr << "Failed to receive message" << std::endl;
            continue;
        }

        std::vector<uchar> jpeg_data(static_cast<unsigned char*>(message.data()),
                                     static_cast<unsigned char*>(message.data()) + message.size());
                                     
        cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        if (img.empty())
        {
            std::cerr << "Failed to decode image" << std::endl;
            continue;
        }

        std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
        double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch()).count();

        ProcessImage(SLAM, img, tframe, pose_publisher, features_publisher, jpeg_data);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
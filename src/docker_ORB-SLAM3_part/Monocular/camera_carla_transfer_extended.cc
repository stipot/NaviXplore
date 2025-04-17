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
                  zmq::socket_t& publisher, const std::vector<uchar>& jpeg_data)
{
    cv::Mat im_copy = im.clone();
    
    Sophus::SE3f pose = SLAM.TrackMonocular(im_copy, tframe);

    if(SLAM.GetTrackingState() == ORB_SLAM3::Tracking::OK)
    {
        Eigen::Matrix4f mat = pose.matrix();
        float tx = mat(0,3);
        float ty = mat(1,3);
        float tz = mat(2,3);

        nlohmann::json root;
        root["timestamp"] = tframe;
        root["image_size"]["width"] = im.cols;
        root["image_size"]["height"] = im.rows;
        root["pose"]["tx"] = tx;
        root["pose"]["ty"] = ty;
        root["pose"]["tz"] = tz;
        root["tracking_status"] = "OK";

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                root["pose_matrix"][i * 4 + j] = mat(i, j);
            }
        }

        std::stringstream jpeg_base64;
        for (size_t i = 0; i < jpeg_data.size(); ++i) {
            jpeg_base64 << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(jpeg_data[i]);
        }
        root["image_data"] = jpeg_base64.str();

        std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
        std::vector<cv::KeyPoint> key_points = SLAM.GetTrackedKeyPointsUn();
        
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
        zmq::message_t message(json_data.size());
        memcpy(message.data(), json_data.data(), json_data.size());
        publisher.send(message, zmq::send_flags::none);
    }
    else
    {
        nlohmann::json root;
        root["timestamp"] = tframe;
        root["tracking_status"] = "LOST";
        root["image_size"]["width"] = im.cols;
        root["image_size"]["height"] = im.rows;

        std::stringstream jpeg_base64;
        for (size_t i = 0; i < jpeg_data.size(); ++i) {
            jpeg_base64 << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(jpeg_data[i]);
        }
        root["image_data"] = jpeg_base64.str();
        
        std::string json_data = root.dump();
        zmq::message_t message(json_data.size());
        memcpy(message.data(), json_data.data(), json_data.size());
        publisher.send(message, zmq::send_flags::none);
        
        std::cout << "Tracking lost!" << std::endl;
    }
}

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_vocabulary> <path_to_settings> "
                  << "<zmq_input_address> <zmq_output_address>" << std::endl;
        return 1;
    }

    const char* vocabulary_path = argv[1];
    const char* settings_path = argv[2];
    const char* zmq_input_address = argv[3];
    const char* zmq_output_address = argv[4];

    ORB_SLAM3::System SLAM(vocabulary_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);

    zmq::context_t context(1);

    zmq::socket_t image_subscriber(context, ZMQ_SUB);
    try {
        image_subscriber.connect(zmq_input_address);
        image_subscriber.set(zmq::sockopt::subscribe, "");
        std::cout << "Connected to image stream at " << zmq_input_address << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect to ZMQ image server: " << e.what() << std::endl;
        return 1;
    }

    zmq::socket_t publisher(context, ZMQ_PUB);
    try {
        publisher.connect(zmq_output_address);
        std::cout << "Publishing all data to " << zmq_output_address << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect publisher: " << e.what() << std::endl;
        return 1;
    }

    while (true)
    {
        zmq::message_t json_message;
        zmq::recv_result_t result = image_subscriber.recv(json_message, zmq::recv_flags::none);
        if (!result) {
            std::cerr << "Failed to receive metadata" << std::endl;
            continue;
        }
        
        std::string json_str(static_cast<char*>(json_message.data()), json_message.size());
        double timestamp = 0.0;
        int frame_id = -1;
        
        try {
            auto json_data = nlohmann::json::parse(json_str);
            timestamp = static_cast<double>(json_data["timestamp"].get<uint64_t>());
            frame_id = json_data["frame_id"].get<int>();
        } catch (std::exception& e) {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            continue;
        }

        zmq::message_t image_message;
        result = image_subscriber.recv(image_message, zmq::recv_flags::none);
        if (!result) {
            std::cerr << "Failed to receive image data" << std::endl;
            continue;
        }

        std::vector<uchar> jpeg_data(static_cast<unsigned char*>(image_message.data()),
                                     static_cast<unsigned char*>(image_message.data()) + image_message.size());
                                     
        cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        if (img.empty())
        {
            std::cerr << "Failed to decode image" << std::endl;
            continue;
        }

        ProcessImage(SLAM, img, timestamp, publisher, jpeg_data);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
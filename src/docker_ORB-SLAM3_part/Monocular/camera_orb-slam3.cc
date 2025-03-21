#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>
#include <zmq.hpp>
#include <vector>
#include <string>
#include <sstream>

void ProcessImage(ORB_SLAM3::System& SLAM, const cv::Mat& im, const double& tframe, zmq::socket_t& publisher)
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
        
        zmq::message_t message(coords_str.size());
        memcpy(message.data(), coords_str.data(), coords_str.size());
        publisher.send(message, zmq::send_flags::none);
    }
    else
    {
        std::cout << "Tracking lost!" << std::endl;
    }
}

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_vocabulary> <path_to_settings> <zmq_image_address> <zmq_coords_address>" << std::endl;
        return 1;
    }

    const char* vocabulary_path = argv[1];
    const char* settings_path = argv[2];
    const char* zmq_image_address = argv[3];
    const char* zmq_coords_address = argv[4];

    ORB_SLAM3::System SLAM(vocabulary_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    try {
        subscriber.connect(zmq_image_address);
        subscriber.set(zmq::sockopt::subscribe, "");
        std::cout << "Connected to image stream at " << zmq_image_address << ". Waiting for images..." << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect to ZMQ image server: " << e.what() << std::endl;
        return 1;
    }
    
    zmq::socket_t publisher(context, ZMQ_PUB);
    try {
        publisher.connect(zmq_coords_address);
        std::cout << "Publishing coordinates to " << zmq_coords_address << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect ZMQ publisher: " << e.what() << std::endl;
        return 1;
    }

    while (true)
    {
        zmq::message_t message;
        zmq::recv_result_t result = subscriber.recv(message, zmq::recv_flags::none);
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

        ProcessImage(SLAM, img, tframe, publisher);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

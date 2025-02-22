#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <System.h>
#include <zmq.hpp>
#include <vector>
#include <cstring>
#include <sstream>
#include <deque>
#include <mutex>
#include <thread>
#include <iomanip>

struct IMUData {
    double timestamp;
    ORB_SLAM3::IMU::Point data;

    IMUData(double t, const ORB_SLAM3::IMU::Point& p) : timestamp(t), data(p) {}
};

class IMUBuffer {
private:
    std::deque<IMUData> buffer;
    double last_frame_timestamp;
    mutable std::mutex mutex;
    const size_t MAX_SIZE = 3000;
    const size_t MIN_IMU_BETWEEN_FRAMES = 6; // ~200Hz/20Hz = 10, чуть меньше для надежности
    const double MAX_TIME_BETWEEN_MEASUREMENTS = 0.01; // 1/100Hz + небольшой запас
    const double EXPECTED_CAMERA_DT = 0.05; // 1/20Hz

    size_t total_imu_measurements;
    size_t total_frames;
    double start_time;

public:
    IMUBuffer() : last_frame_timestamp(0), start_time(0), total_imu_measurements(0), total_frames(0) {}

    void add(double timestamp, const ORB_SLAM3::IMU::Point& point) {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (start_time == 0) {
            start_time = timestamp;
        }
        
        if (!buffer.empty()) {
            double dt = timestamp - buffer.back().timestamp;
            if (dt <= 0) {
                std::cerr << std::fixed << std::setprecision(6)
                         << "Warning: Non-monotonic IMU timestamp detected: " 
                         << buffer.back().timestamp << " -> " << timestamp << std::endl;
                return;
            }
            
            if (dt > MAX_TIME_BETWEEN_MEASUREMENTS) {
                std::cout << std::fixed << std::setprecision(6)
                         << "Warning: Large IMU time gap: " << dt << "s" << std::endl;
            }
        }

        buffer.emplace_back(timestamp, point);
        total_imu_measurements++;

        // Статистика каждые 5 секунд
        if (total_imu_measurements % 1000 == 0) {
            double elapsed = timestamp - start_time;
            std::cout << std::fixed << std::setprecision(2)
                     << "IMU Stats - Rate: " << total_imu_measurements/elapsed 
                     << " Hz, Frame Rate: " << total_frames/elapsed 
                     << " Hz, Buffer size: " << buffer.size() << std::endl;
        }

        while (buffer.size() > MAX_SIZE) {
            buffer.pop_front();
        }
    }

    std::vector<ORB_SLAM3::IMU::Point> getBetweenFrames(double current_frame_timestamp) {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<ORB_SLAM3::IMU::Point> imu_data;
        total_frames++;

        if (buffer.empty()) {
            std::cerr << "Warning: IMU buffer is empty!" << std::endl;
            return imu_data;
        }

        if (last_frame_timestamp == 0) {
            last_frame_timestamp = current_frame_timestamp;
            std::cout << "Initializing last_frame_timestamp to " << std::fixed 
                     << std::setprecision(6) << last_frame_timestamp << std::endl;
            return imu_data;
        }

        double frame_dt = current_frame_timestamp - last_frame_timestamp;
        if (std::abs(frame_dt - EXPECTED_CAMERA_DT) > 0.01) {
            std::cout << "Warning: Unexpected frame interval: " << std::fixed 
                     << std::setprecision(6) << frame_dt 
                     << "s (expected " << EXPECTED_CAMERA_DT << "s)" << std::endl;
        }

        // IMU данные между кадрами
        size_t measurements_count = 0;
        double prev_timestamp = last_frame_timestamp;

        for (const auto& imu_point : buffer) {
            if (imu_point.timestamp > last_frame_timestamp && 
                imu_point.timestamp <= current_frame_timestamp) {
                
                // Проверка интервала между IMU измерениями
                double imu_dt = imu_point.timestamp - prev_timestamp;
                if (imu_dt > MAX_TIME_BETWEEN_MEASUREMENTS) {
                    std::cout << "Warning: Large gap between IMU measurements: " 
                             << std::fixed << std::setprecision(6) << imu_dt << "s" << std::endl;
                }
                
                imu_data.push_back(imu_point.data);
                measurements_count++;
                prev_timestamp = imu_point.timestamp;
            }
        }

        if (measurements_count < MIN_IMU_BETWEEN_FRAMES) {
            std::cout << "Warning: Only " << measurements_count 
                     << " IMU measurements between frames (expected ~10)" << std::endl;
            return std::vector<ORB_SLAM3::IMU::Point>();
        }

        while (!buffer.empty() && buffer.front().timestamp <= current_frame_timestamp) {
            buffer.pop_front();
        }

        std::cout << "Frame interval: " << std::fixed << std::setprecision(6) << frame_dt 
                  << "s, IMU measurements: " << measurements_count 
                  << " (rate: " << measurements_count/frame_dt << " Hz)" << std::endl;

        last_frame_timestamp = current_frame_timestamp;
        return imu_data;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex);
        return buffer.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        buffer.clear();
        last_frame_timestamp = 0;
        total_imu_measurements = 0;
        total_frames = 0;
        start_time = 0;
    }
};

IMUBuffer g_imu_buffer;

void ProcessFrame(ORB_SLAM3::System& SLAM, const cv::Mat& img, 
                 const std::vector<ORB_SLAM3::IMU::Point>& imu_data, double tframe) {
    std::cout << "Processing frame at " << std::fixed << std::setprecision(6) 
              << tframe << " with " << imu_data.size() << " IMU measurements" << std::endl;

    if (!imu_data.empty()) {
        std::cout << "First IMU: t=" << std::fixed << std::setprecision(6) 
                  << imu_data.front().t 
                  << " ax=" << imu_data.front().a[0]
                  << " ay=" << imu_data.front().a[1]
                  << " az=" << imu_data.front().a[2]
                  << " gx=" << imu_data.front().w[0]
                  << " gy=" << imu_data.front().w[1]
                  << " gz=" << imu_data.front().w[2] << std::endl;
        
        std::cout << "Last IMU: t=" << std::fixed << std::setprecision(6) 
                  << imu_data.back().t 
                  << " ax=" << imu_data.back().a[0]
                  << " ay=" << imu_data.back().a[1]
                  << " az=" << imu_data.back().a[2]
                  << " gx=" << imu_data.back().w[0]
                  << " gy=" << imu_data.back().w[1]
                  << " gz=" << imu_data.back().w[2] << std::endl;
    }

    SLAM.TrackMonocular(img, tframe, imu_data);
}

void ProcessIMUData(const std::string& imu_data_str, double timestamp, IMUBuffer& imu_buffer) {
    std::stringstream imu_stream(imu_data_str);
    std::string line;
    std::vector<std::string> imu_lines;

    while (std::getline(imu_stream, line)) {
        if (!line.empty()) {
            imu_lines.push_back(line);
        }
    }

    // Обработка каждой пары строк (акселерометр + гироскоп)
    for (size_t i = 0; i < imu_lines.size(); i += 2) {
        if (i + 1 >= imu_lines.size()) break;

        float ax, ay, az, gx, gy, gz;
        
        if (sscanf(imu_lines[i].c_str(), "Accel: (%f, %f, %f)", &ax, &ay, &az) != 3) {
            std::cerr << "Error parsing accelerometer data: " << imu_lines[i] << std::endl;
            continue;
        }
        
        if (sscanf(imu_lines[i+1].c_str(), "Gyro: (%f, %f, %f)", &gx, &gy, &gz) != 3) {
            std::cerr << "Error parsing gyroscope data: " << imu_lines[i+1] << std::endl;
            continue;
        }

        ORB_SLAM3::IMU::Point imu_point(ax, ay, az, gx, gy, gz, timestamp);
        imu_buffer.add(timestamp, imu_point);
    }
}

int main(int argc, char **argv) {
    if(argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <path_to_vocabulary> <path_to_settings> "
                  << "<camera_zmq_address> <imu_zmq_address>" << std::endl;
        return 1;
    }

    const char* vocabulary_path = argv[1];
    const char* settings_path = argv[2];
    const char* camera_zmq_address = argv[3];
    const char* imu_zmq_address = argv[4];

    std::cout << "Initializing ORB-SLAM3..." << std::endl;
    ORB_SLAM3::System SLAM(vocabulary_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, true);

    zmq::context_t context(1);
    zmq::socket_t camera_subscriber(context, ZMQ_SUB);
    zmq::socket_t imu_subscriber(context, ZMQ_SUB);

    try {
        camera_subscriber.connect(camera_zmq_address);
        camera_subscriber.set(zmq::sockopt::subscribe, "");
        std::cout << "Connected to camera ZMQ server at " << camera_zmq_address << std::endl;

        imu_subscriber.connect(imu_zmq_address);
        imu_subscriber.set(zmq::sockopt::subscribe, "");
        std::cout << "Connected to IMU ZMQ server at " << imu_zmq_address << std::endl;
    } catch (zmq::error_t& e) {
        std::cerr << "Failed to connect to ZMQ servers: " << e.what() << std::endl;
        return 1;
    }

    // Накопление начальных IMU-данных
    std::cout << "Collecting initial IMU data (waiting for 10 seconds)..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    size_t initial_imu_count = 0;
    double first_imu_time = 0;
    double last_imu_time = 0;
    
    while (std::chrono::duration_cast<std::chrono::seconds>(
           std::chrono::steady_clock::now() - start_time).count() < 10) {
        zmq::message_t imu_message;
        if (imu_subscriber.recv(imu_message, zmq::recv_flags::dontwait)) {
            zmq::message_t imu_timestamp_message;
            if (imu_subscriber.recv(imu_timestamp_message, zmq::recv_flags::none)) {
                std::string imu_data_str(static_cast<char*>(imu_message.data()), 
                                       imu_message.size());
                double imu_timestamp = std::stod(std::string(
                    static_cast<char*>(imu_timestamp_message.data()), 
                    imu_timestamp_message.size()));
                
                if (first_imu_time == 0) first_imu_time = imu_timestamp;
                last_imu_time = imu_timestamp;
                
                ProcessIMUData(imu_data_str, imu_timestamp, g_imu_buffer);
                initial_imu_count++;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    double imu_collection_time = last_imu_time - first_imu_time;
    double imu_rate = initial_imu_count / imu_collection_time;
    
    std::cout << "Initial IMU data collected. Buffer size: " << g_imu_buffer.size() 
              << ", Total measurements: " << initial_imu_count 
              << ", Rate: " << std::fixed << std::setprecision(2) 
              << imu_rate << " Hz" << std::endl;

    if (imu_rate < 150.0) {
        std::cerr << "Warning: IMU rate too low! Expected ~200 Hz, got " 
                  << imu_rate << " Hz" << std::endl;
    }

    double last_frame_time = 0;
    size_t frame_count = 0;
    auto processing_start = std::chrono::steady_clock::now();

    while (true) {
        zmq::message_t camera_message;
        if (!camera_subscriber.recv(camera_message, zmq::recv_flags::none)) {
            continue;
        }

        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - processing_start).count();

        if (frame_count % 100 == 0) {
            std::cout << "Average frame rate: " << std::fixed << std::setprecision(2) 
                     << frame_count/elapsed << " Hz" << std::endl;
        }

        std::vector<uchar> jpeg_data(static_cast<unsigned char*>(camera_message.data()),
                                   static_cast<unsigned char*>(camera_message.data()) 
                                   + camera_message.size());
        cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        if (img.empty()) {
            std::cerr << "Failed to decode image" << std::endl;
            continue;
        }

        zmq::message_t camera_timestamp_message;
        if (!camera_subscriber.recv(camera_timestamp_message, zmq::recv_flags::none)) {
            std::cerr << "Failed to receive camera timestamp" << std::endl;
            continue;
        }
        double tframe = std::stod(std::string(static_cast<char*>(camera_timestamp_message.data()), 
                                camera_timestamp_message.size()));

        // Сбор всех доступных IMU-данных перед обработкой кадра
        zmq::message_t imu_message;
        size_t imu_count = 0;
        while (imu_subscriber.recv(imu_message, zmq::recv_flags::dontwait)) {
            zmq::message_t imu_timestamp_message;
            if (imu_subscriber.recv(imu_timestamp_message, zmq::recv_flags::none)) {
                std::string imu_data_str(static_cast<char*>(imu_message.data()), 
                                       imu_message.size());
                double imu_timestamp = std::stod(std::string(
                    static_cast<char*>(imu_timestamp_message.data()), 
                    imu_timestamp_message.size()));
                ProcessIMUData(imu_data_str, imu_timestamp, g_imu_buffer);
                imu_count++;
            }
        }

        // IMU данные между кадрами
        std::vector<ORB_SLAM3::IMU::Point> frame_imu_data = g_imu_buffer.getBetweenFrames(tframe);
        
        if (!frame_imu_data.empty()) {
            ProcessFrame(SLAM, img, frame_imu_data, tframe);
        } else {
            std::cout << "Skipping frame at " << std::fixed << std::setprecision(6) 
                     << tframe << " due to insufficient IMU data" << std::endl;
        }

        // Задержка для стабильной работы
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

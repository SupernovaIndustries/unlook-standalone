// Test program to enumerate available formats from libcamera for IMX296 sensors
#include <iostream>
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/formats.h>

using namespace libcamera;

int main() {
    CameraManager manager;
    int ret = manager.start();
    if (ret) {
        std::cerr << "Failed to start camera manager: " << ret << std::endl;
        return -1;
    }

    auto cameras = manager.cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras found!" << std::endl;
        manager.stop();
        return -1;
    }

    for (size_t i = 0; i < cameras.size(); i++) {
        auto camera = cameras[i];
        std::cout << "\n===== Camera " << i << ": " << camera->id() << " =====" << std::endl;
        
        // Get camera properties
        const ControlList &props = camera->properties();
        if (props.contains(properties::Model.id())) {
            std::cout << "Model: " << props.get(properties::Model).value_or("Unknown") << std::endl;
        }
        
        // Acquire camera
        ret = camera->acquire();
        if (ret) {
            std::cerr << "Failed to acquire camera: " << ret << std::endl;
            continue;
        }
        
        // Generate configurations for different roles
        std::vector<StreamRole> roles = { StreamRole::Raw, StreamRole::VideoRecording, StreamRole::Viewfinder };
        
        for (auto role : roles) {
            std::string roleName;
            switch(role) {
                case StreamRole::Raw: roleName = "Raw"; break;
                case StreamRole::VideoRecording: roleName = "VideoRecording"; break;
                case StreamRole::Viewfinder: roleName = "Viewfinder"; break;
                default: roleName = "Unknown"; break;
            }
            
            std::cout << "\n--- Role: " << roleName << " ---" << std::endl;
            
            auto config = camera->generateConfiguration({ role });
            if (config && !config->empty()) {
                for (unsigned int j = 0; j < config->size(); j++) {
                    StreamConfiguration &cfg = config->at(j);
                    
                    std::cout << "Stream " << j << " default config:" << std::endl;
                    std::cout << "  Size: " << cfg.size.width << "x" << cfg.size.height << std::endl;
                    std::cout << "  Format: " << cfg.pixelFormat.toString() << std::endl;
                    std::cout << "  Stride: " << cfg.stride << std::endl;
                    std::cout << "  Frame size: " << cfg.frameSize << std::endl;
                    
                    // List available formats
                    std::cout << "  Available formats:" << std::endl;
                    for (const auto& format : cfg.formats()) {
                        std::cout << "    - " << format.first.toString() << ": ";
                        bool first = true;
                        for (const auto& size : format.second) {
                            if (!first) std::cout << ", ";
                            std::cout << size.toString();
                            first = false;
                        }
                        std::cout << std::endl;
                    }
                }
            }
        }
        
        // Test specific format configurations
        std::cout << "\n--- Testing specific formats ---" << std::endl;
        
        // Test YUV formats
        std::vector<PixelFormat> testFormats = {
            formats::YUV422,
            formats::YUYV,
            formats::UYVY,
            formats::YUV420,
            formats::NV12,
            formats::NV21,
            formats::SBGGR10,
            formats::SBGGR10_CSI2P,
            formats::SBGGR8,
            formats::RGB888,
            formats::BGR888
        };
        
        auto testConfig = camera->generateConfiguration({ StreamRole::VideoRecording });
        if (testConfig && !testConfig->empty()) {
            StreamConfiguration &cfg = testConfig->at(0);
            
            for (const auto& format : testFormats) {
                cfg.pixelFormat = format;
                cfg.size = { 1456, 1088 };  // IMX296 native resolution
                
                CameraConfiguration::Status status = testConfig->validate();
                std::cout << "Format " << format.toString() << " @ 1456x1088: ";
                
                switch(status) {
                    case CameraConfiguration::Valid:
                        std::cout << "VALID" << std::endl;
                        break;
                    case CameraConfiguration::Adjusted:
                        std::cout << "ADJUSTED to " << cfg.pixelFormat.toString() 
                                  << " @ " << cfg.size.width << "x" << cfg.size.height << std::endl;
                        break;
                    case CameraConfiguration::Invalid:
                        std::cout << "INVALID" << std::endl;
                        break;
                }
            }
        }
        
        camera->release();
    }
    
    manager.stop();
    std::cout << "\nFormat enumeration complete!" << std::endl;
    
    return 0;
}
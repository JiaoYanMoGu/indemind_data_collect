#include <iostream>
#include <thread>
#include <memory>
#include <iomanip>

#include "job_queue.h"
#include "cmdline.hpp"
#include "param_manager.h"
#include "debug_definitions.h"
#include "parameters.h"
#include "DriverInterface.h"

#include <unistd.h>
#include <opencv2/opencv.hpp>

using namespace unicorn;
using namespace std;

typedef std::pair<double, std::string> ImageInfo;
typedef std::pair<std::string, cv::Mat> ImageMetaData;


JobQueue<indem::IMUData> imu_queue;
JobQueue<ImageInfo> image_info_queue;
JobQueue<ImageMetaData> camera_queue;

void ImuCallBackFunction(indem::IMUData *data) {
    INFO("Get new imu frame!");
    imu_queue.Push(*data);
}

void CameraCallBackFunction(indem::cameraData *data) {
    INFO("Get new image frame!");
    std::string image_name = to_string(int(data->_timeStamp)) + string(".png");
    image_info_queue.Push(std::make_pair(data->_timeStamp, image_name));
    camera_queue.Push(std::make_pair(image_name, cv::Mat(data->_height, data->_width, CV_8UC1, data->_image)));
}


void HMDHotplugCallback_func(bool bArrive) {
    if (bArrive) {
        cout << "设备插入\n" << endl;
    } else {
        cout << "设备拔出\n" << endl;
    }
    return;
}

int main(int argc, char **argv) {

    //------------------Parse arg--------------------
    OptionParser paser("Indemind_data_collect, press Q to exit");
    auto help = paser.add<Switch>("h", "help", "use -h to see help!");
    auto save_directory = paser.add<Value<std::string>>("d", "directory", "directoy to save the data", "./");
    auto config_file = paser.add<Value<std::string>>("c", "config", "config file");
    auto thread_num = paser.add<Value<int>>("n", "num_thread", "thread to write image data", 4);
    paser.parse(argc, argv);
    if (help->is_set() || !config_file->is_set()) {
        cout << paser << endl;
        return -1;
    }
    //---------------------------Load configuration --------------------------
    cv::FileStorage config;
    if (!config.open(config_file->value(), cv::FileStorage::READ)) {
        cout << "Open config failed" << endl;
        return -1;
    };
    int ret = 0;
    int width = config["width"];
    int height = config["height"];
    int fps = config["fps"];
    int imu_freq = config["imu_freq"];
    int version = 255;
    size_t info_size = 0;
    unsigned char *module_info = new unsigned char[FLASH_MAX_SIZE];
    ModuleParamInFlash<1> module_param = {0};
    std::shared_ptr<indem::IDriverInterface> driver_ptr(DriverFactory());
    indem::IMAGE_RESOLUTION plan = indem::RESOLUTION_640;


    //-------------------decide the image size and fps----------------------
    if (width == 1280) {
        plan = indem::RESOLUTION_1280;
    } else if (width == 640) {
        plan = indem::RESOLUTION_640;
    } else {
        width = 640;
        plan = indem::RESOLUTION_640;
    }
    if (fps != 25 && fps != 50 && fps != 100 && fps != 200) {
        fps = 50;
    }
    if (fps == 200 && plan != indem::RESOLUTION_640) {
        fps = 100;
    }
    imu_freq = std::min(imu_freq, 1000);
    //-----------------Get module info--------------------------------------
    ret = driver_ptr->GetModuleParams(version, module_info, info_size);
    if (!ret) {
        cout << "Get params failed\n" << endl;
    } else {
        memcpy(&module_param, module_info, info_size);
    }



    //----------------Open the device and set callback function--------------
    ret = driver_ptr->Open(imu_freq, fps, plan);
    if (ret < 0) {
        cout << "Open device error" << endl;
    } else {
        cout << "Open successfully " << endl;
        driver_ptr->SetCameraCallback(CameraCallBackFunction);
        driver_ptr->SetIMUCallback(ImuCallBackFunction);
        SetHotplugCallback(HMDHotplugCallback_func);
    }

    //--------------------------mkdir and touch imu data file-----------------
    int res;
    res=system("mkdir cam0 cam1");
    res=system("touch image.txt imu.txt calibration.txt");
    ofstream imu_file("imu.txt", fstream::out);
    ofstream image_info_file("image.txt", fstream::out);

    // Save calibration info
    ParamManager manager(module_param._parent._camera[0],
                         module_param._parent._camera[1], module_param._parent._imu);
    manager.Save("./calibration.txt");
    //---------------Open new threads to write data to disk---------------------

    std::atomic_bool is_stop(false);

    // thread to save imu data
    std::thread imu_write_thread([&imu_file, &is_stop]() {
        imu_file << "#IMU Data: timestamp(/ms) acc_x(m/s^2) acc_y(m/s^2)  acc_z(m/s^2) gyr_x gyr_y gyr_z" << endl;
        while (!is_stop) {
            imu_file << setprecision(9);
            const auto &imu = imu_queue.Pop();
            if (imu.IsValid()) {
                imu_file << imu.Data()._timeStamp << " " << imu.Data()._acc[0] * _G << " " << imu.Data()._acc[1] * _G
                         << " "
                         << imu.Data()._acc[2] * _G << " " << imu.Data()._gyr[0] << " " << imu.Data()._gyr[1] << " "
                         << imu.Data()._gyr[2] << endl;
            }
        }
        imu_queue.Wait();
    });

    // thread to write image info
    std::thread image_info_write_thread([&image_info_file, &is_stop]() {
        image_info_file << "#ImageInfo: timestamp(/ms) image_name" << endl;
        while (!is_stop) {
            const auto &data = image_info_queue.Pop();
            if (data.IsValid()) {
                image_info_file << data.Data().first << " " << data.Data().second << endl;
            }
        }
        image_info_queue.Wait();
    });

    // threads to write images
    std::vector<std::shared_ptr<thread>> thread_ptrs;
    for (int i = 0; i < thread_num->value(); ++i) {
        thread_ptrs.emplace_back(
                new std::thread([&is_stop]() {
                    while (!is_stop) {
                        const auto &data = camera_queue.Pop();
                        if (!data.IsValid()) continue;
                        const cv::Mat &mat = data.Data().second;
                        const cv::Mat &cam0 = mat.rowRange(0, mat.rows).colRange(0, mat.cols / 2 - 1);
                        const cv::Mat &cam1 = mat.rowRange(0, mat.rows).colRange(mat.cols / 2, mat.cols);
                        cv::imwrite(string("./cam0/" + data.Data().first), cam0);
                        cv::imwrite(string("./cam1/" + data.Data().first), cam1);
                    }
                    camera_queue.Wait();
                })
        );
    }

    imu_write_thread.join();
    image_info_write_thread.join();
    for (const auto &t : thread_ptrs) {
        t->join();
    }
    imu_file.close();
    image_info_file.close();
    return 0;
}

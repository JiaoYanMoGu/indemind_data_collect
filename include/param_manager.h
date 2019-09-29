//
// Created by a409 on 9/29/19.
//

#ifndef INDEMIND_COLLECT_DATA_PARAM_MANAGER_H
#define INDEMIND_COLLECT_DATA_PARAM_MANAGER_H

#include "parameters.h"
#include "debug_definitions.h"
#include <fstream>


using std::endl;

namespace {
    void PrintEach(int row, int col, double *data_ptr, std::ofstream &stream) {
        for (int r = 0; r < row; ++r) {
            for (int c = 0; c < col - 1; ++c) {
                stream << data_ptr[r * col + c] << " ";
            }
            stream << data_ptr[r * col + col - 1] << endl;
        }
    }

    void ReadEach(int row, int col, double *data_ptr, std::ifstream &stream) {
        for (int r = 0; r < row; ++r) {
            for (int c = 0; c < col; ++c) {
                stream >> data_ptr[r * col + c];
            }
        }
    }
} // namespace


class ParamManager {

public:
    std::string config_;
    CameraParameter cam0_;
    CameraParameter cam1_;
    IMUParameter imu_;
public:
    ParamManager() = default;

    explicit ParamManager(const CameraParameter &cam0, const CameraParameter &cam1,
                          const IMUParameter &imu) :
            cam0_(cam0), cam1_(cam1), imu_(imu) {
    }

    explicit ParamManager(const std::string &config)
            : config_(config) {
        Load();
    }

    ~ParamManager() = default;

    bool Load() {
        std::ifstream in_stream(config_);
        if (!in_stream.is_open()) {
            ERROR("Open config file Failed!");
            return false;
        }
        std::string temp;
        getline(in_stream,temp);//#Indemind Calibration File
        in_stream >> temp;//cam0:
        in_stream >> temp >> cam0_._width;//width: width
        in_stream >> temp >> cam0_._height;//height: height
        in_stream >> temp;// T_BS:
        ReadEach(4, 4, cam0_._TSC, in_stream);
        in_stream >> temp;// R:
        ReadEach(3, 3, cam0_._R, in_stream);
        in_stream >> temp; // P:
        ReadEach(3, 4, cam0_._P, in_stream);
        in_stream >> temp; // K:
        ReadEach(3, 3, cam0_._K, in_stream);
        in_stream >> temp; // D;
        ReadEach(1, 4, cam0_._D, in_stream);

        in_stream >> temp;//cam1:
        in_stream >> temp >> cam1_._width;//width: width
        in_stream >> temp >> cam1_._height;//height: height
        in_stream >> temp;// T_BS:
        ReadEach(4, 4, cam1_._TSC, in_stream);
        in_stream >> temp;// R:
        ReadEach(3, 3, cam1_._R, in_stream);
        in_stream >> temp; // P:
        ReadEach(3, 4, cam1_._P, in_stream);
        in_stream >> temp; // K:
        ReadEach(3, 3, cam1_._K, in_stream);
        in_stream >> temp; // D;
        ReadEach(1, 4, cam1_._D, in_stream);

        in_stream >> temp; // IMU:
        in_stream >> temp >> imu_._a_max; // a_max
        in_stream >> temp >> imu_._g_max; //g_max
        in_stream >> temp >> imu_._sigma_g_c; //sigma_g_c
        in_stream >> temp >> imu_._sigma_a_c; //sigma_a_c
        in_stream >> temp >> imu_._sigma_bg; //sigma_bg
        in_stream >> temp >> imu_._sigma_ba; //sigma_ba
        in_stream >> temp >> imu_._sigma_gw_c; //sigma_gw_c
        in_stream >> temp >> imu_._sigma_aw_c; //sigma_aw_c
        in_stream >> temp >> imu_._tau; // tau
        in_stream >> temp >> imu_._g; //g
        in_stream >> temp; // a0:
        ReadEach(1, 4, imu_._a0, in_stream);
        in_stream >> temp; //T_BS
        ReadEach(4, 4, imu_._T_BS, in_stream);
        in_stream >> temp; // Acc
        ReadEach(3, 4, imu_._Acc, in_stream);
        in_stream >> temp;// Gyr
        ReadEach(3, 4, imu_._Gyr, in_stream);
        in_stream.close();
    }

    bool Save(const std::string& config ) {
        std::ofstream of(config);
        if (!of.is_open()) {
            ERROR("Open config file Failed, Save Failed!");
            return false;
        }
        of << "#Indemind Calibration File" << endl;

        // cam0
        of << "cam0:\n";
        of << "width: " << cam0_._width << endl;
        of << "height: " << cam0_._height << endl;
        of << "T_BS:\n";
        PrintEach(4, 4, cam0_._TSC, of);
        of << "R:\n";
        PrintEach(3, 3, cam0_._R, of);
        of << "P: \n";
        PrintEach(3, 4, cam0_._P, of);
        of << "K: \n";
        PrintEach(3, 3, cam0_._K, of);
        of << "D: \n";
        PrintEach(1, 4, cam0_._D, of);


        // cam1
        of << "cam1:\n";
        of << "width: " << cam1_._width << endl;
        of << "height: " << cam1_._height << endl;
        of << "T_BS:\n";
        PrintEach(4, 4, cam1_._TSC, of);
        of << "R:\n";
        PrintEach(3, 3, cam1_._R, of);
        of << "P: \n";
        PrintEach(3, 4, cam1_._P, of);
        of << "K: \n";
        PrintEach(3, 3, cam1_._K, of);
        of << "D: \n";
        PrintEach(1, 4, cam1_._D, of);
        //IMUParameter 结构体说明
        //    a_max: 176.0 # acceleration saturation [m/s^2]
        //    g_max: 7.8 # gyro saturation [rad/s]
        //    sigma_g_c: 12.0e-4 # gyro noise density [rad/s/sqrt(Hz)]
        //    sigma_a_c: 8.0e-3 # accelerometer noise density [m/s^2/sqrt(Hz)]
        //    sigma_bg: 0.03 # gyro bias prior [rad/s]
        //    sigma_ba: 0.1 # accelerometer bias prior [m/s^2]
        //    sigma_gw_c: 4.0e-6 # gyro drift noise density [rad/s^s/sqrt(Hz)]
        //    sigma_aw_c: 4.0e-5 # accelerometer drift noise density [m/s^2/sqrt(Hz)]
        //    tau: 3600.0 # reversion time constant, currently not in use [s]
        //    g: 9.81007 # Earth's acceleration due to gravity [m/s^2]
        //    a0: [ 0.0, 0.0, 0.0 ] # Accelerometer bias [m/s^2]
        //    imu_rate: 200
        //    # tranform Body-Sensor (IMU)
        //    T_BS:
        //        [1.0000, 0.0000, 0.0000, 0.0000,
        //         0.0000, 1.0000, 0.0000, 0.0000,
        //         0.0000, 0.0000, 1.0000, 0.0000,
        //         0.0000, 0.0000, 0.0000, 1.0000]

        // imu
        of << "IMU:\n";
        of << "a_max: " << imu_._a_max << endl;
        of << "g_max: " << imu_._g_max << endl;
        of << "sigma_g_c: " << imu_._sigma_g_c << endl;
        of << "sigma_a_c: " << imu_._sigma_a_c << endl;
        of << "sigma_bg: " << imu_._sigma_bg << endl;
        of << "sigma_ba: " << imu_._sigma_ba << endl;
        of << "sigma_gw_c: " << imu_._sigma_gw_c << endl;
        of << "sigma_aw_c: " << imu_._sigma_aw_c << endl;
        of << "tau: " << imu_._tau << endl;
        of << "g: " << imu_._g << endl;
        of << "a0: " << endl;
        PrintEach(1, 4, imu_._a0, of);
        of << "T_BS: " << endl;
        PrintEach(4, 4, imu_._T_BS, of);
        of << "Acc: " << endl;
        PrintEach(3, 4, imu_._Acc, of);
        of << "Gyr: " << endl;
        PrintEach(3, 4, imu_._Gyr, of);
        of.close();
        return true;
    }

};


#endif //INDEMIND_COLLECT_DATA_PARAM_MANAGER_H

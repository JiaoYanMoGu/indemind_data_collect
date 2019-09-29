/*
IMUParameter 结构体说明
    a_max: 176.0 # acceleration saturation [m/s^2]
    g_max: 7.8 # gyro saturation [rad/s]
    sigma_g_c: 12.0e-4 # gyro noise density [rad/s/sqrt(Hz)]
    sigma_a_c: 8.0e-3 # accelerometer noise density [m/s^2/sqrt(Hz)]
    sigma_bg: 0.03 # gyro bias prior [rad/s]
    sigma_ba: 0.1 # accelerometer bias prior [m/s^2]
    sigma_gw_c: 4.0e-6 # gyro drift noise density [rad/s^s/sqrt(Hz)]
    sigma_aw_c: 4.0e-5 # accelerometer drift noise density [m/s^2/sqrt(Hz)]
    tau: 3600.0 # reversion time constant, currently not in use [s]
    g: 9.81007 # Earth's acceleration due to gravity [m/s^2]
    a0: [ 0.0, 0.0, 0.0 ] # Accelerometer bias [m/s^2]
    imu_rate: 200
    # tranform Body-Sensor (IMU)
    T_BS:
        [1.0000, 0.0000, 0.0000, 0.0000,
         0.0000, 1.0000, 0.0000, 0.0000,
         0.0000, 0.0000, 1.0000, 0.0000,
         0.0000, 0.0000, 0.0000, 1.0000]

*/
#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <type_traits>
#define _G 9.8019967

//2组  左、右目
struct CameraParameter
{
    double _TSC[16];    //4X4 左相机系到传感器坐标系的变换
    int _width;         //图像宽
    int _height;        //图像高

    //distortion_type:equidistant
    double _focal_length[2];//相机fx,fy
    double _principal_point[2];//相机cx,cy

    double _R[9];       //3*3 旋转矩阵
    double _P[12];      //3*4 投影矩阵
    double _K[9];       //3*3 相机内参
    double _D[4];       //4*1 左相机畸变差校正参数,鱼眼畸变
    //--------------------------------------------------------------------
};
//IMU 参数
struct IMUParameter
{
    double _a_max;
    double _g_max;
    double _sigma_g_c;
    double _sigma_a_c;
    double _sigma_bg;
    double _sigma_ba;
    double _sigma_gw_c;
    double _sigma_aw_c;
    double _tau;
    double _g;        //=_G
    double _a0[4];
    double _T_BS[16];
    double _Acc[12]; //加速度计的补偿参数
    double _Gyr[12]; //陀螺仪的补偿参数
};

struct ModuleInfo
{
    char _id[32];
    char _designer[32];
    char _fireware_version[32];
    char _hardware_version[32];
    char _lens[32];
    char _imu[32];
    char _viewing_angle[32];
    char _baseline[32];
};

struct SlamParameter{
    int _numKeyframes;        //关键帧数量 =5
    int   _numImuFrames;      //相邻imu帧数量 =3
    int  _ceres_minIterations;//最小优化次数 =3
    int     _ceres_maxIterations; //最大优化次数 =4
    double     _ceres_timeLimit;    //=3.5000000000000003e-02
    int     detection_threshold; //角点阈值 =30
    int     detection_octaves;  //=0
    int    detection_maxNoKeypoints;//最大角点数量    = 100
    bool displayImages;   //是否显示slam界面  = true
};

//模组所有配置参数
struct ModuleParameters{
    CameraParameter _camera[2]; //左右目相机
    IMUParameter _imu;
    ModuleInfo _device;
    SlamParameter _slam;
};

#define MODULE_SIZE sizeof(ModuleParameters)

template<int VERSION_NUM>
struct ModuleParamInFlash{};

//版本0对应的数据结构，旧版本
template<> struct ModuleParamInFlash<0> {
    ModuleParameters _parent;
};
//版本1对应的数据结构
template<> struct ModuleParamInFlash<1>{
    ModuleParameters _parent;
    int _imgFrequency;          //图像频率
    int _imgResolution;         //图像分辨率
    int _imuFrequency;          //IMU频率
};
#define FLASH_PARAMETER_SIZE_1    sizeof(ModuleParamInFlash<1>)

///////////////////////////////静态断言POD保证新版数据类型能够使用memcpy////////////////////////////////////
static_assert(std::is_pod< ModuleParamInFlash<1> >::value == true, "ModuleParamInFlash<1> is not POD");
static_assert(std::is_pod< ModuleParamInFlash<2> >::value == true, "ModuleParamInFlash<2> is not POD");
static_assert(std::is_pod< ModuleParamInFlash<3> >::value == true, "ModuleParamInFlash<3> is not POD");
static_assert(std::is_pod< ModuleParamInFlash<4> >::value == true, "ModuleParamInFlash<4> is not POD");
static_assert(std::is_pod< ModuleParamInFlash<5> >::value == true, "ModuleParamInFlash<5> is not POD");

#define FLASH_MAX_SIZE  FLASH_PARAMETER_SIZE_1

#endif // PARAMETERS_H

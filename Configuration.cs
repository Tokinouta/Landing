using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    // 导航控制器配置
    public enum GuidanceConfig
    {
        Backstepping = 1, // 1 制导律使用backstepping control
        BacksteppingPPC = 2, //  1 使用基于预设性能，无向量场的反步法制导律
        PPCVectorTimevarying = 3, //  1 制导律使用基于预设性能的时变向量场
        NoPPCVectorNoVarying = 4, //  1 制导律使用无预设性能的固定向量场
        NoPPCVectorTimeVarying = 5, //  1 制导律使用无预设性能的时变向量场
        G3dMPF = 6 //  1 制导律使用3D移动路径跟踪方法
    }

    // 姿态控制器配置
    public enum AttitudeConfig
    {
        Backstepping = 1, //  1 姿态控制律使用Backstepping
        IDLC = 2 //  1 姿态控制律使用直接升力控制
    }

    // 干扰观测器配置
    public enum DisturbanceObserverConfig
    {
        NDO = 1, // 1 使用 nonlinear disturbance observer filter
        FTO = 2, // 1 使用fixed-time disturbance observer filter
        HOSMO = 3, // 1 使用high order sliding mode observer filter
        ESO = 4, // 1 使用ESO观测器
        NONE = 5 // 1 不使用干扰观测器
    }

    // 轨迹配置
    public enum TrajactoryType
    {
        TypeI = 0, // 1 使用x系数为1的轨迹
        TypeII = 1 // 1 使用y，z系数为1的轨迹
    }

    public enum AngularRateConfig
    {
        BS,
        NDI
    }

    public static class Configuration
    {
        public static GuidanceConfig GuidanceController { get; set; } = GuidanceConfig.G3dMPF;
        public static AttitudeConfig AttitudeController { get; set; } = AttitudeConfig.IDLC;
        public static AngularRateConfig AngularRateController { get; set; } = AngularRateConfig.NDI;
        public static DisturbanceObserverConfig DisturbanceObserver { get; set; } = DisturbanceObserverConfig.NDO;

        // 导数滤波器配置参数
        public static bool guidance_command_filter_flag = true; // 1 使用command filter计算导数
        public static bool attitude_command_filter_flag = true; // 1 使用command filter计算导数
        public static bool attitude_tracking_differentiator = false; // 1 使用tracking_differentiator计算导数

        // 轨迹配置
        // 特别注意，更改轨迹类型后，还需更改initial文件中控制器参数
        public static TrajactoryType TrajactoryConfig { get; set; } = TrajactoryType.TypeII;

        // 扰动类型配置
        public static bool DisturbanceTypeI = true; // 1 典型舰尾流扰动

        public static bool wind_enable = false; // 是否使能风场扰动 0:不使能 1:使能
        public static bool deck_enable = false; // 是否使能甲板运动补偿 0:不使能 1:使能

        public static bool L1_adaptive_flag = true;
        // epsilon_gamma = 0.000001;
        // epsilon_kai = 0.000001;
    }
}

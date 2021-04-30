using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.Data.Matlab;

namespace CsharpVersion
{
    public class Disturbance
    {

        // 风场扰动参数

        bool IsWindEnabled = true; // 是否使能风场扰动 0:不使能 1:使能
        int WindCount = 0; // 在dof_update中调用，用于记录到达风场作用区时，经过的仿真步长
        double WindAlpha = 0; // angle of attck caused by wind
        double WindBeta = 0; // new added in mk4.1
        double WindActPosition = -800; // 考虑风场作用的距离，修改风场作用距离，要在此处和.m文件中同时修改
        public bool IsWindDisturbanceStarted = false; // 标记风场扰动开始的参数，1->系统加入风场扰动

        Vector<double> CalCount;
        Matrix<double> WindModelState;
        Vector<double> Wind;
        Vector<double> WindLateral;

        double wind_estimation_NDO;
        double wind_estimation_NDO_lat;

        // 扰动参数
        public double disturbance_kai = 0;
        public double disturbance_gamma = 0;
        public double disturbance_Vk = 0;
        public double disturbance_alpha = 0;
        public double disturbance_p = 0;
        public double disturbance_q = 0;
        public double disturbance_r = 0;


        // Data record, properties(SetAccess = private)
        List<double> d_kai_b2f_record;
        List<double> d_gamma_b2f_record;
        List<double> d_Vk_record;
        List<double[]> d_omega_record;

        public double wind_actual;
        public double wind_actual_lat;

        public Disturbance()
        {
            IsWindEnabled = Configuration.IsWindEnabled;
            if (IsWindEnabled)
            {
                CalCount = MatlabReader.Read<double>("./WindModel.mat", "cal_count").Column(0);
                WindModelState = MatlabReader.Read<double>("./WindModel.mat", "wind_model_state");
                Wind = MatlabReader.Read<double>("./WindModel.mat", "wind").Column(0);
                WindLateral = MatlabReader.Read<double>("./WindModel.mat", "wind_lat").Column(0);
            }
        }

        public void reset()
        {
            disturbance_kai = 0;
            disturbance_gamma = 0;
            disturbance_Vk = 0;
            disturbance_alpha = 0;
            disturbance_p = 0;
            disturbance_q = 0;
            disturbance_r = 0;
            WindCount = 0; // 在dof_update中调用，用于记录到达风场作用区时，经过的仿真步长
            WindAlpha = 0; // angle of attck caused by wind
            WindBeta = 0; // new added in mk4.1
            WindActPosition = -800; // 考虑风场作用的距离，修改风场作用距离，要在此处和.m文件中同时修改
            IsWindDisturbanceStarted = false; // 标记风场扰动开始的参数，1->系统加入风场扰动
        }

        public void updateWind(Plane plane, Ship ship, int step_count)
        {
            if (IsWindEnabled)
            {
                var current_position_ship = ship.Position;
                var current_position = plane.Position;
                var current_Vk = plane.Vk;

                if (Math.Abs(current_position_ship[0] - current_position[0]) < Math.Abs(WindActPosition))
                {
                    IsWindDisturbanceStarted = true;
                    WindAlpha = Wind[step_count - WindCount] / (current_Vk);
                    WindBeta = -WindLateral[step_count - WindCount] / (current_Vk);
                }
                else
                {
                    WindCount = WindCount + 1;
                    WindAlpha = 0;
                    WindBeta = 0;
                }
            }
        }
        // METHOD1 此处显示有关此方法的摘要
        // 此处显示详细说明

        public void updateState(Plane plane)
        {
            double CC_beta = plane.CC_beta;
            double CD_alpha = plane.CD_alpha;
            double CL_beta = plane.CL_beta;
            double CM_alpha1 = plane.CM_alpha1;
            double CM_alpha2 = plane.CM_alpha2;
            double CN_beta = plane.CN_beta;
            double CY_alpha = plane.CY_alpha;
            double current_alpha = plane.Alpha;
            double current_D = plane.D;
            double current_gamma = plane.Gamma;
            double current_miu = plane.Miu;
            double current_Q = plane.Flow;
            double current_Vk = plane.Vk;
            double current_Y = plane.Y;

            double Ixx = plane.PlaneInertia.Ixx;
            double Iyy = plane.PlaneInertia.Iyy;
            double Izz = plane.PlaneInertia.Izz;
            double Ixz = plane.PlaneInertia.Ixz;
            double WingS = plane.PlaneInertia.WingS;
            double WingL = plane.PlaneInertia.WingL;
            double Mass = plane.PlaneInertia.Mass;


            if (IsWindEnabled)
            {
                if (Configuration.UseDisturbanceTypeI)
                {
                    // 典型舰尾流模型
                    disturbance_kai = 1 / (Mass * current_Vk * Cos(current_gamma)) * (current_D * WindAlpha * Sin(current_miu)
                        + current_Q * WingS * (CY_alpha * WindAlpha) * Sin(current_miu)
                        - current_D * WindBeta * Cos(current_miu) + current_Q * WingS * CC_beta * WindBeta * Cos(current_miu));
                    disturbance_gamma = 1 / (Mass * current_Vk) * (current_D * WindAlpha * Cos(current_miu)
                        + current_Q * WingS * (CY_alpha * WindAlpha) * Cos(current_miu)
                        + current_D * WindBeta * Sin(current_miu) - current_Q * WingS * CC_beta * WindBeta * Sin(current_miu));
                    disturbance_Vk = 1 / Mass
                        * (current_Y * WindAlpha - current_Q * WingS * (CD_alpha * WindAlpha));
                    disturbance_alpha = -disturbance_gamma;
                }
                else
                {
                    Console.WriteLine("请指定扰动类型 id41");
                    return;
                }
            }
            else
            {
                disturbance_kai = 0;
                disturbance_gamma = 0;
                disturbance_Vk = 0;
                disturbance_alpha = -disturbance_gamma;
            }

            if (IsWindEnabled)
            {
                if (Configuration.UseDisturbanceTypeI)
                {
                    // 典型舰尾流模型
                    double current_L_wind = current_Q * WingS * WingL * CL_beta * WindBeta;
                    double current_N_wind = current_Q * WingS * WingL * CN_beta * WindBeta;
                    double current_M_wind = current_Q * WingS * plane.CM_alpha * WindAlpha;
                    disturbance_p = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                        * (Izz * current_L_wind + Ixz * current_N_wind);
                    disturbance_r = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                        * (Ixz * current_L_wind + Ixx * current_N_wind);
                    disturbance_q = 1 / Iyy * (current_M_wind);
                }
                else
                {
                    Console.WriteLine("请指定扰动类型 id42");
                    return;
                }
            }
            else
            {
                disturbance_p = 0;
                disturbance_q = 0;
                disturbance_r = 0;
            }
        }

        void record(Plane plane, Ship ship, int step_count)
        {
            double gamma_f = ship.Gamma;
            double gamma_b2f = plane.Gamma - gamma_f;
            // 可以考虑用list存历史数据，但是这种方式可能有性能担忧，
            // 比如需要绘图的时候可能需要新开辟一块内存存数据，以及这样大规模的linq可能也不怎么快
            // 记录元素可以考虑用数组，不用向量
            //List<Vector<double>> rec = new List<Vector<double>>
            //{
            //    cal_count
            //};
            //var query = from r in rec
            //            select r[0];
            //var a = Vector<double>.Build.DenseOfEnumerable(query);
            d_kai_b2f_record.Add((Cos(plane.Gamma) / Cos(gamma_b2f)) * disturbance_kai);
            d_gamma_b2f_record.Add(disturbance_gamma);
            d_Vk_record.Add(disturbance_Vk);
            d_omega_record.Add(new[] { disturbance_p, disturbance_q, disturbance_r });

            if (IsWindEnabled)
            {
                if (Math.Abs(ship.Position[0] - plane.Position[0]) < Math.Abs(WindActPosition))
                {
                    //wind_actual(end + 1) = wind(step_count - wind_count); // 实际风速
                    //wind_actual_lat(end + 1) = wind_lat(step_count - wind_count); // 实际风速
                }
                else
                {
                    //wind_actual(end + 1) = 0;
                    //wind_actual_lat(end + 1) = 0;
                }
            }
        }
    }
}

using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public class Disturbance
    {

        // 风场扰动参数

        bool wind_enable = true; // 是否使能风场扰动 0:不使能 1:使能
        int wind_count = 0; // 在dof_update中调用，用于记录到达风场作用区时，经过的仿真步长
        double current_alpha_wind = 0; // angle of attck caused by wind
        double current_beta_wind = 0; // new added in mk4.1
        double wind_act_position = -800; // 考虑风场作用的距离，修改风场作用距离，要在此处和.m文件中同时修改
        public bool wind_disturbance_start = false; // 标记风场扰动开始的参数，1->系统加入风场扰动

        Vector<double> cal_count;
        Vector<double> wind_model_state;
        Vector<double> wind;
        Vector<double> wind_lat;

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

        // wind_estimation_NDO;
        // wind_estimation_NDO_lat;
        public double wind_actual;
        public double wind_actual_lat;

        public Disturbance()
        {
            wind_enable = Configuration.wind_enable;
            if (wind_enable)
            {
                // load_system('wind_model');
                //[cal_count, wind_model_state, wind, wind_lat] = sim('wind_model');
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
            wind_count = 0; // 在dof_update中调用，用于记录到达风场作用区时，经过的仿真步长
            current_alpha_wind = 0; // angle of attck caused by wind
            current_beta_wind = 0; // new added in mk4.1
            wind_act_position = -800; // 考虑风场作用的距离，修改风场作用距离，要在此处和.m文件中同时修改
            wind_disturbance_start = false; // 标记风场扰动开始的参数，1->系统加入风场扰动
        }

        //void AddListener(fpl flightPathLoop)
        //{
        //    //addlistener(flightPathLoop, "RecordFlightPathLoopVarEvent", @updateWindSpeedEventHandler);
        //}

        //void updateWindSpeedEventHandler(EventArgs e)
        //{
        //    wind_estimation_NDO = e.data[4];
        //    wind_estimation_NDO_lat = e.data[5];
        //}

        public void updateWind(Plane plane, Ship ship, int step_count)
        {
            if (wind_enable)
            {
                var current_position_ship = ship.current_position_ship;
                var current_position = plane.current_position;
                var current_Vk = plane.current_Vk;

                if (Math.Abs(current_position_ship[0] - current_position[0]) < Math.Abs(wind_act_position))
                {
                    wind_disturbance_start = true;
                    current_alpha_wind = wind[step_count - wind_count] / (current_Vk);
                    current_beta_wind = -wind_lat[step_count - wind_count] / (current_Vk);
                }
                else
                {
                    wind_count = wind_count + 1;
                    current_alpha_wind = 0;
                    current_beta_wind = 0;
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
            double current_alpha = plane.current_alpha;
            double current_D = plane.current_D;
            double current_gamma = plane.current_gamma;
            double current_miu = plane.current_miu;
            double current_Q = plane.current_Q;
            double current_Vk = plane.current_Vk;
            double current_Y = plane.current_Y;
            double CY_alpha = plane.CY_alpha;
            double Ixx = Plane.Ixx;
            double Ixz = Plane.Ixz;
            double Iyy = Plane.Iyy;
            double Izz = Plane.Izz;
            double m = Plane.m;
            double wing_L = Plane.wing_L;
            double wing_S = Plane.wing_S;

            if (wind_enable)
            {
                if (Configuration.DisturbanceTypeI)
                {
                    // 典型舰尾流模型
                    // previous_disturbance_gamma = disturbance_gamma;
                    disturbance_kai = 1 / (m * current_Vk * Cos(current_gamma)) * (current_D * current_alpha_wind * Sin(current_miu)
                        + current_Q * wing_S * (CY_alpha * current_alpha_wind) * Sin(current_miu)
                        - current_D * current_beta_wind * Cos(current_miu) + current_Q * wing_S * CC_beta * current_beta_wind * Cos(current_miu));
                    disturbance_gamma = 1 / (m * current_Vk) * (current_D * current_alpha_wind * Cos(current_miu)
                        + current_Q * wing_S * (CY_alpha * current_alpha_wind) * Cos(current_miu)
                        + current_D * current_beta_wind * Sin(current_miu) - current_Q * wing_S * CC_beta * current_beta_wind * Sin(current_miu));
                    disturbance_Vk = 1 / m
                        * (current_Y * current_alpha_wind - current_Q * wing_S * (CD_alpha * current_alpha_wind));
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

            if (wind_enable)
            {
                if (Configuration.DisturbanceTypeI)
                {
                    // 典型舰尾流模型
                    double current_L_wind = current_Q * wing_S * wing_L * CL_beta * current_beta_wind;
                    double current_N_wind = current_Q * wing_S * wing_L * CN_beta * current_beta_wind;
                    double current_M_wind = current_Q * wing_S * (CM_alpha2 * current_alpha + CM_alpha1) * current_alpha_wind;
                    disturbance_p = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                        * (Izz * current_L_wind
                        + Ixz * current_N_wind);
                    disturbance_r = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                        * (Ixz * current_L_wind
                        + Ixx * current_N_wind);
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
            double gamma_f = ship.gamma_s;
            double gamma_b2f = plane.current_gamma - gamma_f;
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
            d_kai_b2f_record.Add((Cos(plane.current_gamma) / Cos(gamma_b2f)) * disturbance_kai);
            d_gamma_b2f_record.Add(disturbance_gamma);
            d_Vk_record.Add(disturbance_Vk);
            d_omega_record.Add(new[] { disturbance_p, disturbance_q, disturbance_r });

            if (wind_enable)
            {
                if (Math.Abs(ship.current_position_ship[0] - plane.current_position[0]) < Math.Abs(wind_act_position))
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

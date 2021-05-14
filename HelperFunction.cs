using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using static MathNet.Numerics.Constants;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ModelEntities.Enumerations;
using ModelEntities;

namespace CsharpVersion
{
    /// <summary>
    /// 辅助函数，用于特定计算场景
    /// </summary>
    public static class HelperFunction
    {
        /// <summary>
        /// 仿真配置
        /// </summary>
        public static Configuration Configuration { get; set; }

        /// <summary>
        /// 计算理想下滑道下的着舰点，仅计算y，z坐标
        /// </summary>
        /// <param name="position_a">飞机当前位置</param>
        /// <param name="position_s">航母当前位置</param>
        /// <param name="theta_s">斜角甲板角度</param>
        /// <param name="psi_s">航母偏转角度</param>
        /// <returns>理想着舰点</returns>
        public static Vector<double> ideal_path(Vector<double> position_a, Vector<double> position_s, double theta_s, double psi_s) // 甲板坐标系下飞机x坐标
        {
            // y_z[0] = 0;                                    // 理想下滑道y坐标
            //     y_z[1] = -Tan(3.5 * Pi / 180) * x + 21.1;      // 理想下滑道z坐标
            //     y_z[1] = Tan(3.5 * Pi / 180) * (position_a[0] - position_s[0]) - 21.1;      // 理想下滑道z坐标

            //     y_z[0] = (position_s[0] - position_a[0]) * Tan(9 * Pi / 180);// 理想下滑道y坐标
            //     y_z[1] = Tan(3.5 * Pi / 180) * (position_a[0] - position_s[0]);      // 理想下滑道z坐标

            //     y_z[0] = (position_s[0] - position_a[0]) * Tan(abs(theta_s) + abs(psi_s));// 理想下滑道y坐标
            //     y_z[1] = position_s[1] + Tan(3.5 * Pi / 180) * ((position_a[0] - position_s[0]) / cos(abs(theta_s) + abs(psi_s)));      // 理想下滑道z坐标

            double psi_total = -theta_s + psi_s;
            Vector<double> delta = position_s - position_a;
            //double delta_x = position_s[0] - position_a[0];
            //double delta_y = position_s[1] - position_a[1];
            double l1 = delta[0] / Cos(psi_total);
            double l2 = (delta[1] - delta[0] * Tan(psi_total)) * Sin(psi_total);
            double l = l1 + l2;
            double yd;
            double zd;
            if (l > 1620)
            {
                zd = 1620 * Tan(-3.5 * Pi / 180);
                yd = position_s[1] - 1620 * Sin(psi_total);
            }
            else
            {
                zd = 1620 * Tan(-3.5 * Pi / 180);
                yd = position_s[1] - 1620 * Sin(psi_total);
            }
            //y_z[0] = yd;
            //y_z[1] = zd;
            return Vector<double>.Build.Dense(new[] { yd, zd });
        }

        /// <summary>
        /// 【说实话这个算的是个啥我也不知道】
        /// </summary>
        /// <param name="position_a">飞机当前位置</param>
        /// <param name="position_s">航母当前位置</param>
        /// <returns>一个跟轨迹追踪有关系的东西</returns>
        public static Vector<double> vector_filed_trac(Vector<double> position_a, Vector<double> position_s)
        {
            Vector<double> trac_err = Vector<double>.Build.Dense(2);
            if (Configuration.TrajactoryConfig == TrajactoryType.TypeI)
            {
                // 轨迹方程模式I
                trac_err[0] = position_a[0] + 1 / Tan(9 * Pi / 180) * position_a[1] - position_s[0];
                // trac_err[1] = position_a[0] + 1 / Tan(-3.5 * Pi / 180) * position_a[2] + 21.1 / Tan(-3.5 * Pi / 180) - position_s[0];
                trac_err[1] = position_a[0] + 1 / Tan(-3.5 * Pi / 180) * position_a[2] + 0 / Tan(-3.5 * Pi / 180) - position_s[0];
            }
            else if (Configuration.TrajactoryConfig == TrajactoryType.TypeII)
            {
                // 轨迹方程模式II
                trac_err[0] = Tan(9 * Pi / 180) * position_a[0] + position_a[1] - Tan(9 * Pi / 180) * position_s[0];
                // trac_err[1] = Tan(-3.5 * Pi / 180) * position_a[0] + position_a[2] + 21.1 - Tan(-3.5 * Pi / 180) * position_s[0];
                trac_err[1] = Tan(-3.5 * Pi / 180) * position_a[0] + position_a[2] + 0 - Tan(-3.5 * Pi / 180) * position_s[0];
            }
            else
            {
                return null;
            }
            return trac_err;
        }
    }
}

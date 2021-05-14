using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    /// <summary>
    /// 传递数据更新时用到的变量
    /// </summary>
    public class XChangedEventArgs : EventArgs
    {
        /// <summary>
        /// 仿真时间步长
        /// </summary>
        public double Dt { get; set; }
        /// <summary>
        /// 变量变化量
        /// </summary>
        public Vector<double> Data { get; set; }

        /// <summary>
        /// 构造变化参数实例
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        /// <param name="data">变量变化量</param>
        public XChangedEventArgs(double dt, Vector<double> data)
        {
            Dt = dt;
            Data = data;
        }
    }
}

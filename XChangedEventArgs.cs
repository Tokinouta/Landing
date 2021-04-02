using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public class XChangedEventArgs : EventArgs
    {
        public double Dt { get; set; }
        public Vector<double> Data { get; set; }

        public XChangedEventArgs(double dt, Vector<double> data)
        {
            Dt = dt;
            Data = data;
        }
    }
}

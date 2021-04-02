using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion.PositionController
{
    interface IPositionController
    {
        public Plane Plane { get; set; }
        public Ship Ship { get; set; }
        Vector<double> CalculateOutput(double dt, double current_time, int step_count);
        void InvokeRecordEvent();
        void Reset();
    }
}

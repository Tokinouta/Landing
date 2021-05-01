using MathNet.Numerics.LinearAlgebra;
using ModelEntities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    interface IControlModule
    {
        Configuration Configuration { get; }

        void CalculateState(double dt, Vector<double> input);

        void CalculateObservation();

        void CalculateNonlinearObserver(double dt, Disturbance disturbance);

        void CalculateOutput();

        void CalculateLimiter(double dt);

        void CalculateFilter(double dt);

        void UpdateState(double dt, Disturbance disturbance);

        void Record(double dt);

        void Reset();
    }
}

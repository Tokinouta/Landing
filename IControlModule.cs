using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    interface IControlModule
    {
        void calculateState(double dt, Vector<double> input);
        void calculateState(double dt, Vector<double> input, Plane plane);

        void calculateObservation(Plane plane);

        void calculateNonlinearObserver(double dt, Disturbance disturbance);

        void calculateOutput();

        void calculateLimiter(double dt, Plane plane);

        void calculateFilter(double dt, Plane plane);

        void updateState(double dt, Plane plane, Disturbance disturbance);

        void record();

        void reset(Plane plane);
    }
}

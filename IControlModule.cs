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

        void calculateObservation();

        void calculateNonlinearObserver(double dt, Disturbance disturbance);

        void calculateOutput();

        void calculateLimiter(double dt);

        void calculateFilter(double dt);

        void updateState(double dt, Disturbance disturbance);

        void record();

        void reset();
    }
}

using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Diagnostics;

// CUDA support is not available in Math.NET

namespace CsharpVersion
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");
            //Control.UseNativeCUDA();
            string s = Control.Describe();
            Console.WriteLine(s);
            var m = Matrix<double>.Build.Random(500, 500);
            var v = Vector<double>.Build.Random(500);

            var w = Stopwatch.StartNew();
            var y1 = m.Solve(v);
            Console.WriteLine(w.Elapsed);
            Console.WriteLine(y1);

            // Using the Intel MKL native provider
            Control.UseNativeMKL();
            s = Control.Describe();
            Console.WriteLine(s);

            w.Restart();
            var y2 = m.Solve(v);
            Console.WriteLine(w.Elapsed);
            Console.WriteLine(y2);

            w.Restart();
            var y3 = m.Solve(v);
            Console.WriteLine(w.Elapsed);
            Console.WriteLine(y3);
            Ship ship = new Ship();
            Plane pl = new Plane(ship);
            Console.WriteLine("rarara");

            Simulation sim = new Simulation();
            w.Restart();
            sim.simulate();
            Console.WriteLine(w.Elapsed);
            Console.WriteLine(sim.plane.current_position.ToString("G40"));
        }
    }
}

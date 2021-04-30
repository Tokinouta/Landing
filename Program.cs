using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

// CUDA support is not available in Math.NET

namespace CsharpVersion
{
    class Program
    {
        static void Main(string[] args)
        {
            Control.UseNativeMKL();
            Simulation sim = new();
            Console.WriteLine(Thread.CurrentThread.ManagedThreadId);
            var w = Stopwatch.StartNew();
            sim.Simulate();
            Console.WriteLine(w.Elapsed);
            //Thread.Sleep(5000);
            Console.WriteLine(sim.Plane.Position.ToString("G40"));
        }
    }
}

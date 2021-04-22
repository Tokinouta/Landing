using MathNet.Numerics.Data.Matlab;
using MathNet.Numerics.LinearAlgebra;
using Microsoft.AspNetCore.SignalR.Client;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using HistoryDemo;
using HistoryDemo.Entities;

namespace CsharpVersion
{
    class Simulation
    {
        HubConnection connection;
        double frequency = 400; // 计算频率 Hz
        double dt; // 1 / f
        int step_count = 0;
        double current_time = 0;

        public Ship Ship { get; set; }
        public Plane Plane { get; set; }
        public Disturbance Disturbance { get; set; }
        public AttitudeLoop AttitudeLoop { get; set; }
        public FlightPathLoop FlightPathLoop { get; set; }
        public PositionLoop PositionLoop { get; set; }
        public AngularRateLoop AngularRateLoop { get; set; }
        public SimulationRecord Record { get; set; }

        public Simulation()
        {
            InitializeInitialParameters();

            Ship = new Ship();
            Plane = new Plane(Ship);
            Disturbance = new Disturbance();
            PositionLoop = new PositionLoop(Plane, Ship);
            FlightPathLoop = new FlightPathLoop(Plane, Ship);
            AttitudeLoop = new AttitudeLoop(Plane, Ship);
            AngularRateLoop = new AngularRateLoop(Plane, Ship);
            Console.WriteLine("text");
            // plane.addListeners(positionLoop, flightPathLoop, attitudeLoop, angularRateLoop);
            Record = new SimulationRecord();

            Plane.X1ChangedEvent += PositionLoop.OnUpdateState;
            Plane.X2ChangedEvent += FlightPathLoop.OnUpdateState;
            Plane.X3ChangedEvent += AttitudeLoop.OnUpdateState;
            Plane.X4ChangedEvent += AngularRateLoop.OnUpdateState;
            Plane.RecordPlaneStateEvent += Record.OnRecordPlaneState;
            //disturbance.AddListener(flightPathLoop);
            // addlistener(app, "simulationStep", "PostSet", @modifyFrequency);
        }

        public void Simulate()
        {
            //fid = fopen('out.txt', 'a');
            // load('position.mat');
            // while (step_count < 2000)
            connection = new HubConnectionBuilder()
                .WithUrl("https://localhost:5001/chatHub")
                .Build();
            connection.StartAsync();
            Matrix<double> X1_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "X1_record");

            Matrix<double> X2_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "X2_record");

            Matrix<double> X3_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "X3_record");

            Matrix<double> X4_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "X4_record");

            Matrix<double> derive_X1_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "derive_X1_record");

            Matrix<double> position_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "position_record");

            Matrix<double> position_ship_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "position_ship_record");

            Matrix<double> u1_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "u1_record");

            Matrix<double> u2_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "u2_record");

            Matrix<double> u3_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "u3_record");

            Matrix<double> uact_record = MatlabReader.Read<double>(@"E:\大学课程文件\毕业设计\Experimental code\CsharpVersion\matlab.mat", "uact_record");

            var ini = new Initialization()
            {
                InitialPositionX = Plane.Position[0],
                InitialPositionY = Plane.Position[1],
                InitialPositionZ = Plane.Position[2],
                InitialAttitudePhi = Plane.Phi,
                InitialAttitudePsi = Plane.Psi,
                InitialAttitudeTheta = Plane.Theta
            };
            var conf = new HistoryDemo.Entities.Configuration()
            {
                GuidanceConfig = (HistoryDemo.Entities.GuidanceConfig)Configuration.GuidanceController
            };


            while ((Plane.Position[0] - Ship.Position[0]) < 0)
            {
                // t1 = position_record(step_count,:);
                // con = sum(t1.* t1) ~= sum(plane.current_position.* plane.current_position);
                SingleStep();//fclose(fid);
                if (step_count % 50 == 0)
                {
                    Task.Run(() =>
                    {
                        Task.Delay(250);
                        connection.InvokeAsync("SendData", "user", Plane.Alpha);
                    });
                }
                //if (!position_record.Row(step_count - 1).Equals(plane.current_position))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!position_ship_record.Row(step_count - 1).Equals(ship.current_position_ship))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!derive_X1_record.Row(step_count - 1).Equals(positionLoop.derive_X1))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!X1_record.Row(step_count - 1).Equals(positionLoop.current_X1))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!u1_record.Row(step_count - 1).Equals(positionLoop.current_u1))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!X2_record.Row(step_count - 1).Equals(flightPathLoop.current_X2))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!u2_record.Row(step_count - 1).Equals(flightPathLoop.current_u2))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!X3_record.Row(step_count - 1).Equals(attitudeLoop.current_X3))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!u3_record.Row(step_count - 1).Equals(attitudeLoop.current_u3))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!X4_record.Row(step_count - 1).Equals(angularRateLoop.current_X4))
                //{
                //    Console.WriteLine(step_count);
                //}
                //if (!uact_record.Row(step_count - 1).Equals(angularRateLoop.filter_Uact))
                //{
                //    Console.WriteLine(step_count);
                //}

            }
            Console.WriteLine(step_count);
            string fileName = $"datalog\\{DateTime.Now:yyyy-MM-dd-HH-mm-ss}.mat";
            Record.SaveToFile(fileName);
            using var db = new AppDbContext();
            var t = db.Configurations.First();
            var b = t == conf;
            var bi = new BasicInformation()
            {
                //Id = 1, 
                DateTime = DateTime.Now,
                SimConfiguration = conf,
                SimInitialization = ini,
                PathToData = fileName
            };
            //var t = db.Configurations.Find(new HistoryDemo.Entities.Configuration(){
            //    GuidanceConfig = (HistoryDemo.Entities.GuidanceConfig)Configuration.GuidanceController
            //});
            db.Add(bi);
            db.Add(ini);
            db.Add(conf);
            db.SaveChanges();
        }

        void SingleStep()
        {
            step_count++;
            current_time = dt * step_count;

            PositionLoop.CalculateObservation();
            FlightPathLoop.CalculateObservation();
            AttitudeLoop.CalculateObservation();
            AngularRateLoop.CalculateObservation();

            FlightPathLoop.CalculateNonlinearObserver(dt, Disturbance);
            AngularRateLoop.CalculateNonlinearObserver(dt, Disturbance);

            PositionLoop.calculatePrescribedParameter();
            Ship.calculateCompensation(dt, PositionLoop, step_count);
            PositionLoop.CalculateState(dt, null);
            PositionLoop.calculateOutput(dt, current_time, step_count);
            PositionLoop.CalculateLimiter(dt);

            FlightPathLoop.CalculateState(dt, PositionLoop.U1);
            FlightPathLoop.CalculateOutput();
            FlightPathLoop.CalculateLimiter(dt);
            FlightPathLoop.CalculateFilter(dt);

            AttitudeLoop.CalculateState(dt, FlightPathLoop.U2);
            AttitudeLoop.CalculateOutput();
            AttitudeLoop.CalculateLimiter(dt);

            AngularRateLoop.CalculateState(dt, AttitudeLoop.U3);
            AngularRateLoop.CalculateOutput();
            AngularRateLoop.calculateLimiter(dt, step_count);
            AngularRateLoop.CalculateFilter(dt);

            Plane.UpdateState(dt, Disturbance);
            Ship.updateState(dt);
            Disturbance.updateWind(Plane, Ship, step_count);

            Plane.Record();
            Ship.record();
            PositionLoop.Record(dt);
            FlightPathLoop.Record(dt);
            AttitudeLoop.Record(dt);
            AngularRateLoop.Record(dt);
            //Task.Run(() => Console.WriteLine(step_count));
            //record.time_record.Add(current_time);
        }

        void Reset()
        {
            step_count = 0;
            current_time = 0;
            Ship.reset();
            Plane.Reset(Ship);
            PositionLoop.Reset();
            FlightPathLoop.Reset();
            AttitudeLoop.Reset();
            AngularRateLoop.Reset();
            Disturbance.reset();
        }

        void PlotData()
        {
            //notify(obj, "PlotEvent")
        }

        void InitializeInitialParameters()
        {
            dt = 1 / frequency; // 1 / f
        }

        void ModifyFrequency()
        {
            //dt = e.AffectedObject.simulationStep;
            //frequency = 1 / dt;
            //disp(dt);
        }
    }
}

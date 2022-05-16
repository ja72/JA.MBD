using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Globalization;

namespace JA
{
    using System.Diagnostics;
    using JA.Dynamics;
    internal class Program
    {
        const double pi = Math.PI;
        const double deg = Math.PI / 180;
        static void Main(string[] args)
        {
            var box = Shapes.Box(
                Vector3.Zero,
                Quaternion.Identity,
                δX: 0.05,
                δY: 0.12,
                δΖ: 0.002);

            var body = new RigidBody(0.09, box)
            {
                InitialPosition = Vector3.Zero,
                InitialOrientation = Quaternion.FromAxisAngle(Axis.X, 90 * deg) * Quaternion.FromAxisAngle(Axis.Y, 1 * deg),
            };

            body.SetDensity(7600);

            Console.WriteLine(body);

            var I = body.BodyInertia.GetDiagonal();
            Console.WriteLine($"MMoi = {I}");

            double ω = 1000 * pi;
            body.InitialVelocity = Vector3.Zero;
            body.InitialOmega = ω * Vector3.UnitX;

            var sim = new Simulation(body)
            {
                Gravity = 0 * Vector3.UnitY
            };
            double angDelta = 1.0 * deg;
            double endTime = 1;
            int steps = -(int)(-ω * endTime / angDelta);

            TestSimulation(sim, endTime, steps);

            //BenchSimulation(sim);
            // Laptop, 125 kops (kilo steps per second)
            // Home, 250 kops (kilo steps per second)

            //AccuracyTest(sim);

        }
        static void TestSimulation(Simulation sim, double endTime, int steps)
        {
            Console.WriteLine($"Test MBD Simulation to {endTime} seconds in {steps} steps.");
            var sw = new Stopwatch();
            sim.Step += (s, ev) =>
            {
                if (ev.Frame % (steps / 25) == 0 || ev.Frame == steps)
                {
                    sw.Stop();
                    AddValue(ev.Frame, 7);
                    AddValue(ev.Time, "f7", 10);
                    for (int i = 0; i < ev.Count; i++)
                    {
                        var state = ev.Current[i];
                        var rb = ev.Bodies[i];
                        AddValue(state.Orientation, "f4", 36);
                        var m = state.GetMotion(rb);
                        AddValue(m.omg, "f4", 34);
                        AddValue(state.AngularMomentum, "f4", 24);
                    }
                    Console.WriteLine();
                    sw.Start();
                }
            };
            AddColumn("Frame", 7);
            AddColumn("Time", 10);
            for (int i = 0; i < sim.Count; i++)
            {
                AddColumn("Orientation", 36);
                AddColumn("Omega", 34);
                AddColumn("Angular", 24);
            }
            Console.WriteLine();
            sim.Reset();
            sw.Start();
            sim.Run(endTime, steps);
            sw.Stop();

            Console.WriteLine($"Elapsed Time = {sw.Elapsed.TotalSeconds:g4} sec.");
        }
        static void BenchSimulation(Simulation sim)
        {
            sim.ClearEvents();
            sim.Reset();
            sim.Run(0.02, 2000);
            var sw = new Stopwatch();
            var rb = sim.Bodies.First();
            int steps = 47200;
            double endTime = 0.1;
            Console.WriteLine($"Bench MBD Simulation to {endTime} seconds in {steps} steps.");
            Console.WriteLine($"{"Steps",8} {"Time",12} {"kops",14} {"|Omg|", 12}");
            for (int i = 0; i < 17; i++)
            {
                sim.Reset();
                sw.Restart();
                sim.Run(endTime, steps);
                sw.Stop();
                var state = sim.Current.First();
                var bm = state.GetMotion(rb);
                Console.WriteLine($"{steps,8} {sw.Elapsed.TotalSeconds,12:g4} {steps / sw.Elapsed.TotalSeconds / 1000,14:g4} {bm.omg.Magnitude.ToString("f3"),12}");

                steps = -(int)(- steps * 1.2);
            }
            // Home, 250 kops (kilo steps per second) is consistently typical.
        }

        static void AccuracyTest(Simulation sim)
        {
            sim.ClearEvents();
            var omg0 = sim.Bodies[0].InitialOmega.Magnitude;
            int steps = 8600;
            double endTime = 1;
            Console.Write(" ");
            AddColumn("Steps", 8);
            AddColumn("Orientation", 38);
            AddColumn("Omega", 38);
            Console.WriteLine();
            var sw = new Stopwatch();
            var results = new List<(int s, double t, Quaternion q, Vector3 omg)>();
            for (int i = 0; i < 12; i++)
            {
                sim.Reset();
                sw.Restart();
                sim.Run(endTime, steps);
                sw.Stop();
                var state = sim.Current[0];
                Console.Write(" ");
                AddValue(steps, 8);
                AddValue(state.Orientation, "f4", 38);
                var (vee, omg) = state.GetMotion(sim.Bodies[0]);
                results.Add((steps, sw.Elapsed.TotalSeconds, state.Orientation, omg));
                AddValue(omg, "f4", 38);
                Console.WriteLine();

                steps = (int)(steps * 1.5f);
            }
            var best = results.Last();
            Console.WriteLine();
            AddColumn("Steps", 8);
            AddColumn("Time", 12);
            AddColumn("kops", 12);
            AddColumn("Resoltion", 12);
            AddColumn("|Ori|", 12);
            AddColumn("Angle", 12);
            AddColumn("%Rot", 12);
            Console.WriteLine();
            for (int i = 0; i < results.Count; i++)
            {
                Quaternion δq = best.q / results[i].q;
                Vector3 δω = (results[i].omg - best.omg);
                var (axis, angle) = δq.GetAxisAngle();
                AddValue(results[i].s, 8);
                AddValue(results[i].t, "f3", 12);
                AddValue(results[i].s / results[i].t / 1000, "f3", 12);
                var dt = endTime / results[i].s;
                var res = omg0 * dt / deg;
                AddValue(res, "g3", 12);
                AddValue(results[i].q.Magnitude, "g4", 12);
                AddValue(angle / deg, "f4", 12);
                AddValue(δω.Magnitude / omg0, "f4", 12);
                Console.WriteLine();
            }
        }
        static void AddColumn(string name, int width)
        {
            Console.Write($" {name.PadLeft(width)}");
        }
        static void AddValue(IFormattable value, int width)
            => AddValue(value, "g", width);
        static void AddValue(IFormattable value, string formatting, int width)
        {
            Console.Write($" {value.ToString(formatting, CultureInfo.InvariantCulture.NumberFormat).PadLeft(width)}");
        }

    }
}


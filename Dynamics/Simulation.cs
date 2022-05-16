using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA.Dynamics
{
    public class StepEventArgs : EventArgs
    {
        public StepEventArgs(Simulation simulation, double timeStep)
        {
            Frame = simulation.Frame;
            Time = simulation.Time;
            TimeStep = timeStep;
            Current = simulation.Current;
            Bodies = simulation.Bodies;
        }
        public StepEventArgs(int frame, double time, State[] current, RigidBody[] bodies, double timeStep)
        {
            Frame = frame;
            Time = time;
            TimeStep = timeStep;
            Current = current;
            Bodies = bodies;
        }

        public int Frame { get; }
        public double Time { get; }
        public double TimeStep { get; }
        public State[] Current { get; }
        public RigidBody[] Bodies { get; }
        public int Count { get => Current.Length; }
    }

    public class Simulation
    {
        public event EventHandler<StepEventArgs> Step;
        public Simulation(params RigidBody[] bodies)
        {
            Gravity = Vector3.Zero;
            Bodies = bodies;

            Reset();
        }
        public Vector3 Gravity { get; set; }
        public int Frame { get; set; }
        public double Time { get; set; }
        public State[] Current { get; set; }
        public RigidBody[] Bodies { get; }
        public int Count { get => Current.Length; }

        public void ClearEvents()
        {
            if (Step == null) return;
            foreach (var h in Step.GetInvocationList())
            {
                Step -= (EventHandler<StepEventArgs>)h;
            }
        }
        public void Reset()
        {
            Frame = 0;
            Time = 0;
            Current = Bodies.Select((rb) => rb.GetInitialState()).ToArray();
            OnStep(0);
        }
        public void Run(double endTime, double timeStep)
        {
            int steps = (int)Math.Ceiling((endTime - Time) / timeStep);
            Run(endTime, steps);
        }
        public void Run(double endTime, int steps)
        {
            double timeStep = (endTime - Time) / steps;
            while (Time < endTime - timeStep / 128)
            {
                double h = Math.Min(timeStep, endTime - Time);
                Current = Integrate(Current, ref h);
                Time += h;
                Frame++;
                OnStep(h);
            }
        }

        public virtual (Vector3 force, Vector3 torque)[] GetForces(double time, State[] current)
        {
            return current.Select((y, i) => (Bodies[i].Mass * Gravity, Vector3.Zero)).ToArray();
        }
        State[] GetRate(double time, State[] current)
        {
            var f = GetForces(time, current);
            return current.Select((y, i) => y.Rate(Bodies[i], f[i].force, f[i].torque)).ToArray();
        }
        State[] SingleStep(State[] current, double timeStep, State[] rate)
        {
            return current.Select((y, i) => y.Step(timeStep, rate[i])).ToArray();
        }

        public State[] Integrate(State[] current, ref double timeStep)
        {
            var k0 = GetRate(Time, current);
            var k1 = GetRate(Time + timeStep / 2, SingleStep(current, timeStep / 2, k0)).ToArray();
            var k2 = GetRate(Time + timeStep / 2, SingleStep(current, timeStep / 2, k1)).ToArray();
            var k3 = GetRate(Time, SingleStep(current, timeStep, k2)).ToArray();

            double h6 = timeStep / 6, h3 = timeStep / 3;

            var next = new State[current.Length];
            for (int i = 0; i < next.Length; i++)
            {
                next[i] = new State(
                    current[i].Position + h6 * k0[i].Position + h3 * k1[i].Position + h3 * k2[i].Position + h6 * k3[i].Position,
                    Quaternion.Normalize(current[i].Orientation + h6 * k0[i].Orientation + h3 * k1[i].Orientation + h3 * k2[i].Orientation + h6 * k3[i].Orientation),
                    current[i].Momentum + h6 * k0[i].Momentum + h3 * k1[i].Momentum + h3 * k2[i].Momentum + h6 * k3[i].Momentum,
                    current[i].AngularMomentum + h6 * k0[i].AngularMomentum + h3 * k1[i].AngularMomentum + h3 * k2[i].AngularMomentum + h6 * k3[i].AngularMomentum);
            }
            return next;
        }
        protected void OnStep(double timeStep)
        {
            Step?.Invoke(this, new StepEventArgs(this, timeStep));
        }
    }

}

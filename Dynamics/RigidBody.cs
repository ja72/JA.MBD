using System;
using System.Collections.Generic;
using System.Linq;
using JA.Dynamics;

namespace JA.Dynamics
{
    public class RigidBody
    {
        public RigidBody(double mass, params Shape[] shapes)
        {
            Mass = mass;
            Shapes = new List<Shape>(shapes);

            double V = 0;
            Vector3 cg = Vector3.Zero;
            Matrix3 I = Matrix3.Zero;
            for (int i = 0; i < shapes.Length; i++)
            {
                double dV = shapes[i].GetVolume();
                V += dV;
                cg += dV * shapes[i].Position;
                I += dV*shapes[i].GetBodyInertia();
            }
            CenterOfMass = cg / V;
            Density = mass / V;
            BodyInertia = Density * I;
            // Move MMOI to center of mass
            var cgx = CenterOfMass.CrossOp();
            BodyInertia += mass * cgx * cgx;
            InverseBodyInertia = BodyInertia.Inverse();

            InitialPosition = Vector3.Zero;
            InitialOrientation = Quaternion.Identity;
            InitialVelocity = Vector3.Zero;
            InitialOmega = Vector3.Zero;
        }

        public IReadOnlyList<Shape> Shapes { get; }
        public double Mass { get; private set; }
        public double Volume { get; }
        public double Density { get; private set; }
        public Vector3 CenterOfMass { get; }
        public Matrix3 BodyInertia { get; private set; }
        public Matrix3 InverseBodyInertia { get; private set; }
        public void SetDensity(double ρ)
        {
            double f = ρ / Density;
            Density = ρ;
            Mass *= f;
            BodyInertia *= f;
            InverseBodyInertia /= f;
        }

        public Vector3 InitialPosition { get; set; }
        public Quaternion InitialOrientation { get; set; }
        public Vector3 InitialVelocity { get; set; }
        public Vector3 InitialOmega { get; set; }

        public Matrix3 GetInertiaMatrix(Quaternion orientation, bool inverse = false)
        {
            var R = orientation.ToRotation();
            if (inverse)
            {
                return R * InverseBodyInertia * R.Transpose();
            }
            else
            {
                return R * BodyInertia * R.Transpose();
            }
        }

        public State GetInitialState()
        {
            return State.FromBodyMotion(this, InitialPosition, InitialOrientation, InitialVelocity, InitialOmega);
        }

        public void SetInitialState(State state)
        {
            InitialPosition = state.Position;
            InitialOrientation = state.Orientation;
            var (vee, omg) = state.GetMotion(this);
            InitialVelocity = vee;
            InitialOmega = omg;
        }

        public override string ToString()
        {
            return $"{string.Join(",",Shapes)}, Mass={Mass}, at {InitialPosition}";
        }
    }
}


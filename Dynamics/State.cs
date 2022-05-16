using JA.Dynamics;

namespace JA.Dynamics
{
    public readonly struct State
    {

        readonly (Vector3 pos, Quaternion ori, Vector3 mom, Vector3 ang) data;

        #region Factory
        public State(Vector3 pos, Quaternion ori, Vector3 mom, Vector3 ang)
        {
            this.data = (pos, ori, mom, ang);
        }

        public static State FromBodyMotion(RigidBody body, Vector3 pos, Quaternion ori, Vector3 vee, Vector3 omg) 
        {
            double m = body.Mass;
            Matrix3 R = ori.ToRotation();
            Vector3 c = R * body.CenterOfMass;
            Matrix3 IC = R * body.BodyInertia * R.Transpose();
            Vector3 v_cg = vee + (omg ^ c);
            Vector3 mom = m * v_cg;
            Vector3 ang = IC * omg + (c ^ mom);

            return new State(pos, ori, mom, ang);
        }

        #endregion

        #region Properties
        public Vector3 Position { get => data.pos; }
        public Quaternion Orientation { get => data.ori; }
        public Vector3 Momentum { get => data.mom; }
        public Vector3 AngularMomentum { get => data.ang; }

        public (Vector3 vee, Vector3 omg) GetMotion(RigidBody body)
        {
            double m = body.Mass;
            Matrix3 R = Orientation.ToRotation();
            Vector3 c = R * body.CenterOfMass;
            Matrix3 MC = R * body.InverseBodyInertia * R.Transpose();
            Vector3 omg = MC * (AngularMomentum - (c ^ Momentum));
            Vector3 vee = Momentum / m + (c ^ omg);
            return (vee, omg);
        }
        #endregion

        #region Dynamics
        public State Step(double h, State yp)
            => new State(
                data.pos + h * yp.data.pos,
                data.ori + h * yp.data.ori,
                data.mom + h * yp.data.mom,
                data.ang + h * yp.data.ang);
        public State Rate(RigidBody body, Vector3 force, Vector3 torque)
        {
            var (vee, omg) = GetMotion(body);
            var q = data.ori;
            var qp = Quaternion.Product(0.5 * omg, q);
            return new State(vee, qp, force, torque);
        }
        public State NormalizeOrientation()
        {
            return new State(
                Position,
                Quaternion.Normalize(Orientation),
                Momentum,
                AngularMomentum);
        }

        #endregion

        #region Algebra

        public static State Add(State a, State b)
            => new State(
                a.data.pos + b.data.pos,
                a.data.ori + b.data.ori,
                a.data.mom + b.data.mom,
                a.data.ang + b.data.ang);
        public static State Subtract(State a, State b)
            => new State(
                a.data.pos - b.data.pos,
                a.data.ori - b.data.ori,
                a.data.mom - b.data.mom,
                a.data.ang - b.data.ang);

        public static State Scale(double f, State a)
            => new State(
                f * a.data.pos,
                f * a.data.ori,
                f * a.data.mom,
                f * a.data.ang);
        public static State Negate(State a)
            => new State(
                -a.data.pos,
                -a.data.ori,
                -a.data.mom,
                -a.data.ang);

        #endregion

        #region Operators

        public static State operator +(State a, State b) => Add(a, b);
        public static State operator -(State a, State b) => Subtract(a, b);
        public static State operator -(State a) => Negate(a);
        #endregion
    }
}


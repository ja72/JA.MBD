using System;
using System.Runtime.CompilerServices;

namespace JA.Dynamics
{
    public readonly struct Quaternion : IEquatable<Quaternion>, IFormattable
    {
        readonly (Vector3 v, double s) data;

        #region Factory
        public Quaternion(Vector3 vector, double scalar)
        {
            this.data = (vector, scalar);
        }
        public static readonly Quaternion Zero = new Quaternion(Vector3.Zero, 0);
        public static readonly Quaternion Identity = new Quaternion(Vector3.Zero, 1);

        public static implicit operator Quaternion(Vector3 vector)
            => new Quaternion(vector, 0);
        public static explicit operator Vector3(Quaternion quaternion)
            => quaternion.Vector;
        public static explicit operator double(Quaternion quaternion)
            => quaternion.Scalar;
        public static Quaternion FromAxisAngle(Vector3 axis, double angle)
            => new Quaternion(Vector3.Normalize(axis) * Math.Sin(angle / 2), Math.Cos(angle / 2));
        public static Quaternion FromRotation(Matrix3 rotation)
        {
            double x = rotation.A32 - rotation.A23;
            double y = rotation.A13 - rotation.A31;
            double z = rotation.A21 - rotation.A12;
            double t = rotation.A11 + rotation.A22 + rotation.A33;
            double s = 0.5 * Math.Sqrt((x * x + y * y + z * z) / (3 - t));
            double f = 1 / (4 * s);
            Vector3 v = new Vector3(f * x, f * y, f * z);
            return new Quaternion(v, s);
        }
        #endregion

        #region Properties
        public int Size { get => 4; }
        public Vector3 Vector => data.v;
        public double Scalar => data.s;
        public double Magnitude => Math.Sqrt(SumSquares(this));
        public Quaternion ToUnit() => Normalize(this);

        public Quaternion Conjugate()
            => new Quaternion(-data.v, data.s);

        public Quaternion Inverse()
            => Conjugate() / SumSquares(this);

        public Matrix3 ToRotation(bool inverse = false)
        {
            int inv = inverse ? -1 : 1;
            Matrix3 vx = data.v.CrossOp();
            return 1 + 2 * (inv * data.s * vx + vx * vx);
        }
        public Vector3 Rotate(Vector3 vector, bool inverse = false)
        {
            int inv = inverse ? -1 : 1;
            Vector3 vxp = Vector3.Cross(data.v, vector);
            Vector3 vxvxp = Vector3.Cross(data.v, vxp);
            return vector + 2 * (inv * data.s * vxp + vxvxp);
        }

        public (Vector3 axis, double angle) GetAxisAngle()
        {
            var u = Math.Sqrt(1 - data.s * data.s);
            var axis = data.v / u;
            var angle = 2 * Math.Asin(u);
            return (axis, angle);
        }
        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SumSquares(Quaternion quatertion)
        {
            return Vector3.SumSquares(quatertion.data.v) + quatertion.data.s * quatertion.data.s;
        }

        public static Quaternion Normalize(Quaternion quaternion)
        {
            double m2 = SumSquares(quaternion);
            if (m2 > 0 && m2!=1)
            {
                return Scale(1 / Math.Sqrt(m2), quaternion);
            }
            return quaternion;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion Add(Quaternion a, Quaternion b)
            => new Quaternion(a.data.v + b.data.v, a.data.s + b.data.s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion Subtract(Quaternion a, Quaternion b)
            => new Quaternion(a.data.v - b.data.v, a.data.s - b.data.s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion Scale(double f, Quaternion a)
            => new Quaternion(f * a.data.v, f * a.data.s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion Negate(Quaternion a)
            => new Quaternion(-a.data.v, -a.data.s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Quaternion quaternion, Quaternion other)
            => Vector3.Dot(quaternion.data.v, other.data.v) + quaternion.data.s * other.data.s;
        public static Quaternion Product(Quaternion a, Quaternion b)
        {
            return new Quaternion(
                a.data.s * b.data.v + b.data.s * a.data.v + Vector3.Cross(a.data.v, b.data.v),
                a.data.s * b.data.s - Vector3.Dot(a.data.v, b.data.v));
        }
        public static Quaternion Divide(Quaternion a, Quaternion b)
        {
            return a * b.Inverse();
        }
        #endregion

        #region Operators
        public static Quaternion operator +(Quaternion a, Quaternion b) => Add(a, b);
        public static Quaternion operator -(Quaternion a) => Negate(a);
        public static Quaternion operator -(Quaternion a, Quaternion b) => Subtract(a, b);
        public static Quaternion operator *(double a, Quaternion b) => Scale(a, b);
        public static Quaternion operator *(Quaternion a, double b) => Scale(b, a);
        public static Quaternion operator /(Quaternion a, double b) => Scale(1 / b, a);
        public static Quaternion operator *(Quaternion a, Quaternion b) => Product(a, b);
        public static Quaternion operator /(Quaternion a, Quaternion b) => Divide(a, b);

        #endregion

        #region Formatting
        public override string ToString() => ToString("g");
        public string ToString(string formatting) => ToString(formatting, null);
        public string ToString(string format, IFormatProvider provider)
        {
            return $"<{Vector.ToString(format, provider)}|{Scalar.ToString(format, provider)}>";
        }
        #endregion

        #region Equality
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is Quaternion quaternion
                && Equals(quaternion);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Quaternion other)
            => data.Equals(other.data);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return 1768953197 + data.GetHashCode();
        }

        public static bool operator ==(Quaternion left, Quaternion right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Quaternion left, Quaternion right)
        {
            return !(left == right);
        }

        #endregion
    }
}


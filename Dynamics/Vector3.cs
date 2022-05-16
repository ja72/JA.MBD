using System;
using System.Runtime.CompilerServices;

namespace JA.Dynamics
{
    public enum Axis
    {
        X,Y,Z,
    }
    public readonly struct Vector3 : IEquatable<Vector3>, IFormattable
    {
        readonly (double x, double y, double z) data;

        #region Factory
        public Vector3(double x, double y, double z)
        {
            this.data = (x, y, z);
        }
        public static readonly Vector3 Zero = new Vector3(0, 0, 0);
        public static readonly Vector3 UnitX = new Vector3(1, 0, 0);
        public static readonly Vector3 UnitY = new Vector3(0, 1, 0);
        public static readonly Vector3 UnitZ = new Vector3(0, 0, 1);

        public static Vector3 FromAxis(Axis axis)
        {
            switch (axis)
            {
                case Axis.X: return UnitX;
                case Axis.Y: return UnitY;
                case Axis.Z: return UnitZ;
                default:
                    throw new NotSupportedException($"Axis {axis} not supported.");
            }
        }
        public static implicit operator Vector3(Axis axis) => FromAxis(axis);

        #endregion

        #region Properties
        public int Size { get => 3; }

        public double X => data.x;
        public double Y => data.y;
        public double Z => data.z;
        public double Magnitude => Math.Sqrt(SumSquares(this));
        public Vector3 ToUnit() => Normalize(this); 
        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SumSquares(Vector3 vector)
        {
            return vector.data.x * vector.data.x + vector.data.y * vector.data.y + vector.data.z * vector.data.z;
        }

        public static Vector3 Normalize(Vector3 vector)
        {
            double m2 = SumSquares(vector);
            if (m2 > 0)
            {
                return Scale(1 / Math.Sqrt(m2), vector);
            }
            return vector;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Add(Vector3 a, Vector3 b)
            => new Vector3(a.data.x + b.data.x, a.data.y + b.data.y, a.data.z + b.data.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Subtract(Vector3 a, Vector3 b)
            => new Vector3(a.data.x - b.data.x, a.data.y - b.data.y, a.data.z - b.data.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Scale(double f, Vector3 a)
            => new Vector3(f * a.data.x, f * a.data.y, f * a.data.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Negate(Vector3 a)
            => new Vector3(-a.data.x, -a.data.y, -a.data.z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double Dot(Vector3 vector, Vector3 other)
            => vector.data.x * other.data.x + vector.data.y * other.data.y + vector.data.z * other.data.z;
        public static Vector3 Cross(Vector3 vector, Vector3 other)
          => new Vector3(
              vector.data.y * other.data.z - vector.data.z * other.data.y,
              vector.data.z * other.data.x - vector.data.x * other.data.z,
              vector.data.x * other.data.y - vector.data.y * other.data.x);

        public Matrix3 CrossOp() => Matrix3.Cross(this);
        #endregion

        #region Operators
        public static Vector3 operator +(Vector3 a, Vector3 b) => Add(a, b);
        public static Vector3 operator -(Vector3 a) => Negate(a);
        public static Vector3 operator -(Vector3 a, Vector3 b) => Subtract(a, b);
        public static Vector3 operator *(double a, Vector3 b) => Scale(a, b);
        public static Vector3 operator *(Vector3 a, double b) => Scale(b, a);
        public static Vector3 operator /(Vector3 a, double b) => Scale(1 / b, a);
        public static double operator *(Vector3 a, Vector3 b) => Dot(a, b);
        public static Vector3 operator ^(Vector3 a, Vector3 b) => Cross(a, b);

        #endregion

        #region Formatting
        public override string ToString() => ToString("g");
        public string ToString(string formatting) => ToString(formatting, null);
        public string ToString(string format, IFormatProvider provider)
        {
            return $"({X.ToString(format, provider)},{Y.ToString(format, provider)},{Z.ToString(format, provider)})";
        }
        #endregion

        #region Equality
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is Vector3 vector
                && Equals(vector);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector3 other)
            => data.Equals(other.data);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return 1768953197 + data.GetHashCode();
        }

        public static bool operator ==(Vector3 left, Vector3 right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Vector3 left, Vector3 right)
        {
            return !(left == right);
        }

        #endregion
    }
}


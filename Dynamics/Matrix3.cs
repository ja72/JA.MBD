using System;
using System.Runtime.CompilerServices;

namespace JA.Dynamics
{
    public readonly struct Matrix3 : IEquatable<Matrix3>
    {
        readonly (double a11, double a12, double a13,
            double a21, double a22, double a23,
            double a31, double a32, double a33) data;

        #region Factory

        public Matrix3(double a11, double a12, double a13,
            double a21, double a22, double a23,
            double a31, double a32, double a33)
        {
            this.data = (a11, a12, a13, a21, a22, a23, a31, a32, a33);
        }
        public static readonly Matrix3 Zero = new Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0);
        public static readonly Matrix3 Identity = new Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);

        public static Matrix3 FromRows(Vector3 row1, Vector3 row2, Vector3 row3)
            => new Matrix3(
                row1.X, row1.Y, row1.Z,
                row2.X, row2.Y, row2.Z,
                row3.X, row3.Y, row3.Z);
        public static Matrix3 FromColumns(Vector3 column1, Vector3 column2, Vector3 column3)
            => new Matrix3(
                column1.X, column2.X, column3.X,
                column1.Y, column2.Y, column3.Y,
                column1.Z, column2.Z, column3.Z);

        public static Matrix3 Scalar(double a) => new Matrix3(a, 0, 0, 0, a, 0, 0, 0, a);
        public static implicit operator Matrix3(double a) => Scalar(a);
        public static Matrix3 Diagonal(double a11, double a22, double a33) => new Matrix3(a11, 0, 0, 0, a22, 0, 0, 0, a33);
        public static Matrix3 Symmetric(double a11, double a22, double a33, double a12, double a13, double a23)
            => new Matrix3(a11, a12, a13, a12, a22, a23, a13, a23, a33);

        public static Matrix3 Diagonal(Vector3 vector)
            => Diagonal(vector.X, vector.Y, vector.Z);
        public static Matrix3 Cross(Vector3 vector)
            => new Matrix3(0, -vector.Z, vector.Y,
                vector.Z, 0, -vector.X,
                -vector.Y, vector.X, 0);
        #endregion

        #region Properties
        public int Rows { get => 3; }
        public int Columns { get => 3; }

        public double A11 { get => data.a11; }
        public double A12 { get => data.a12; }
        public double A13 { get => data.a13; }
        public double A21 { get => data.a21; }
        public double A22 { get => data.a22; }
        public double A23 { get => data.a23; }
        public double A31 { get => data.a31; }
        public double A32 { get => data.a32; }
        public double A33 { get => data.a33; }

        public Vector3 GetRow(int row)
        {
            switch (row)
            {
                case 0: return new Vector3(data.a11, data.a12, data.a13);
                case 1: return new Vector3(data.a21, data.a22, data.a23);
                case 2: return new Vector3(data.a31, data.a32, data.a33);
                default:
                    throw new ArgumentOutOfRangeException(nameof(row));
            }
        }
        public Vector3 GetColumn(int column)
        {
            switch (column)
            {
                case 0: return new Vector3(data.a11, data.a21, data.a31);
                case 1: return new Vector3(data.a12, data.a22, data.a32);
                case 2: return new Vector3(data.a13, data.a23, data.a33);
                default:
                    throw new ArgumentOutOfRangeException(nameof(column));
            }
        }

        public Vector3 GetDiagonal() => new Vector3(data.a11, data.a22, data.a33);

        #endregion

        #region Algebra
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Add(Matrix3 matrix, Matrix3 other)
            => new Matrix3(
                matrix.data.a11 + other.data.a11, matrix.data.a12 + other.data.a12, matrix.data.a13 + other.data.a13,
                matrix.data.a21 + other.data.a21, matrix.data.a22 + other.data.a22, matrix.data.a23 + other.data.a23,
                matrix.data.a31 + other.data.a31, matrix.data.a32 + other.data.a32, matrix.data.a33 + other.data.a33);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]        
        public static Matrix3 Subtract(Matrix3 matrix, Matrix3 other)
            => new Matrix3(
                matrix.data.a11 - other.data.a11, matrix.data.a12 - other.data.a12, matrix.data.a13 - other.data.a13,
                matrix.data.a21 - other.data.a21, matrix.data.a22 - other.data.a22, matrix.data.a23 - other.data.a23,
                matrix.data.a31 - other.data.a31, matrix.data.a32 - other.data.a32, matrix.data.a33 - other.data.a33);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Scale(double factor, Matrix3 matrix)
            => new Matrix3(
            factor * matrix.data.a11, factor * matrix.data.a12, factor * matrix.data.a13,
            factor * matrix.data.a21, factor * matrix.data.a22, factor * matrix.data.a23,
            factor * matrix.data.a31, factor * matrix.data.a32, factor * matrix.data.a33);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Negate(Matrix3 matrix)
            => new Matrix3(
            -matrix.data.a11, -matrix.data.a12, -matrix.data.a13,
            -matrix.data.a21, -matrix.data.a22, -matrix.data.a23,
            -matrix.data.a31, -matrix.data.a32, -matrix.data.a33);
        public static Vector3 Product(Matrix3 matrix, Vector3 vector)
            => new Vector3(
                Vector3.Dot(matrix.GetRow(0), vector),
                Vector3.Dot(matrix.GetRow(1), vector),
                Vector3.Dot(matrix.GetRow(2), vector));
        public static Vector3 Product(Vector3 vector, Matrix3 matrix)
            => new Vector3(
                Vector3.Dot(vector, matrix.GetColumn(0)),
                Vector3.Dot(vector, matrix.GetColumn(1)),
                Vector3.Dot(vector, matrix.GetColumn(2)));
        public static Matrix3 Product(Matrix3 matrix, Matrix3 other)
        {
            Vector3 row1 = matrix.GetRow(0);
            Vector3 row2 = matrix.GetRow(1);
            Vector3 row3 = matrix.GetRow(2);
            Vector3 column1 = other.GetColumn(0);
            Vector3 column2 = other.GetColumn(1);
            Vector3 column3 = other.GetColumn(2);

            return new Matrix3(
                           Vector3.Dot(row1, column1), Vector3.Dot(row1, column2), Vector3.Dot(row1, column3),
                           Vector3.Dot(row2, column1), Vector3.Dot(row2, column2), Vector3.Dot(row2, column3),
                           Vector3.Dot(row3, column1), Vector3.Dot(row3, column2), Vector3.Dot(row3, column3));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3 Transpose() 
            => new Matrix3(
                data.a11, data.a21, data.a31,
                data.a12, data.a22, data.a32,
                data.a13, data.a23, data.a33);

        public Matrix3 Inverse()
        {
            var d = data.a11 * (data.a22 * data.a33 - data.a23 * data.a32) 
                + data.a12 * (data.a23 * data.a31 - data.a21 * data.a33)
                + data.a13 * (data.a21 * data.a32 - data.a22 * data.a31);
            var id = 1 / d;
            return new Matrix3(
                id*(data.a22 * data.a33 - data.a23 * data.a32), 
                id*(data.a13 * data.a32 - data.a12 * data.a33), 
                id*(data.a12 * data.a23 - data.a13 * data.a22),
                id*(data.a23 * data.a31 - data.a21 * data.a33), 
                id*(data.a11 * data.a33 - data.a13 * data.a31), 
                id*(data.a13 * data.a21 - data.a11 * data.a23),
                id*(data.a21 * data.a32 - data.a22 * data.a31), 
                id*(data.a12 * data.a31 - data.a11 * data.a32), 
                id*(data.a11 * data.a22 - data.a12 * data.a21));
        }

        public Vector3 Solve(Vector3 vector) => Inverse() * vector;
        public Matrix3 Solve(Matrix3 other) => Inverse() * other;
        #endregion

        #region Operators
        public static Matrix3 operator +(Matrix3 a, Matrix3 b) => Add(a, b);
        public static Matrix3 operator -(Matrix3 a, Matrix3 b) => Subtract(a, b);
        public static Matrix3 operator -(Matrix3 b) => Negate(b);
        public static Matrix3 operator *(double a, Matrix3 b) => Scale(a, b);
        public static Matrix3 operator *(Matrix3 a, double b) => Scale(b, a);
        public static Vector3 operator *(Matrix3 a, Vector3 b) => Product(a, b);
        public static Vector3 operator *(Vector3 a, Matrix3 b) => Product(a, b);
        public static Matrix3 operator *(Matrix3 a, Matrix3 b) => Product(a, b);
        public static Matrix3 operator /(Matrix3 a, double b) => Scale(1 / b, a);
        public static Matrix3 operator ~(Matrix3 b) => b.Transpose();
        public static Matrix3 operator !(Matrix3 b) => b.Inverse();
        public static Vector3 operator /(Vector3 a, Matrix3 b) => b.Solve(a);
        public static Matrix3 operator /(Matrix3 a, Matrix3 b) => b.Solve(a);

        #endregion

        #region Equality
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is Matrix3 matrix &&
                   Equals(matrix);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Matrix3 matrix)
            => data.Equals(matrix.data);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return 1768953197 + data.GetHashCode();
        }

        public static bool operator ==(Matrix3 left, Matrix3 right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Matrix3 left, Matrix3 right)
        {
            return !(left == right);
        }
        #endregion

        #region Formatting
        public override string ToString() => ToString("g");
        public string ToString(string formatting) => ToString(formatting, null);
        public string ToString(string format, IFormatProvider provider)
        {
            return $"[{data.a11.ToString(format, provider)},{data.a12.ToString(format, provider)},{data.a13.ToString(format, provider)}|" + Environment.NewLine +
                $"{data.a21.ToString(format, provider)},{data.a22.ToString(format, provider)},{data.a23.ToString(format, provider)}|" + Environment.NewLine +
                $"{data.a31.ToString(format, provider)},{data.a32.ToString(format, provider)},{data.a33.ToString(format, provider)}]";
        }
        #endregion
    }
}


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA.Dynamics
{

    public static class Shapes
    {
        public static Shape Sphere(Vector3 position, double radius)
            => new Shape.SphereShape(position, Quaternion.Identity, radius);
        public static Shape Box(Vector3 position, Quaternion orientation, double δX, double δY, double δΖ)
            => new Shape.CuboidShape(position, orientation, δX, δY, δΖ);
        public static Shape Plate(Vector3 position, Quaternion orientation, double δX, double δY)
            => new Shape.CuboidShape(position,orientation, δX, δY, 0);
        public static Shape Cylinder(Vector3 position, Quaternion orientation, double length, double radius)
            => new Shape.CylinderShape(position, orientation, length, radius);
        public static Shape Disk(Vector3 position, Quaternion orientation, double radius)
            => new Shape.CylinderShape(position, orientation, 0, radius);
        public static Shape Rod(Vector3 position, Quaternion orientation, double length)
            => new Shape.CylinderShape(position, orientation, length, 0);
    }
    public abstract class Shape
    {
        protected Shape(Vector3 position, Quaternion orientation)
        {
            Position = position;
            Orientation = orientation;
        }
        public Vector3 Position { get; set; }
        public Quaternion Orientation { get; set; }

        public Matrix3 GetBodyInertia()
        {
            var (I_1, I_2, I_3) = GetUnitMmoi();
            var I = Matrix3.Diagonal(I_1, I_2, I_3);
            var R = Orientation.ToRotation();
            // orient mmoi about body axis
            I = R * I * R.Transpose();
            var dx = Position.CrossOp();
            // Parallel axis theoreom
            return I - GetVolume() * dx * dx;
        }

        public abstract double GetVolume();
        public abstract (double I_1, double I_2, double I_3) GetUnitMmoi();
        public abstract override string ToString();

        internal class SphereShape : Shape
        {
            public SphereShape(Vector3 position, Quaternion orientation, double radius)
                : base(position,orientation)
            {
                Radius = radius;
            }

            public double Radius { get; set; }
            public override double GetVolume() => 4 * Math.PI * Radius * Radius * Radius / 3;
            public override (double I_1, double I_2, double I_3) GetUnitMmoi() => (2 * Radius * Radius / 5, 2 * Radius * Radius / 5, 2 * Radius * Radius / 5);
            public override string ToString() => $"Sphere({Radius})";
        }
        internal class CuboidShape : Shape
        {
            public CuboidShape(Vector3 position, Quaternion orientation, double δX, double δY, double δΖ)
                : base(position, orientation)
            {
                ΔX = δX;
                ΔY = δY;
                ΔΖ = δΖ;
            }

            public double ΔX { get; }
            public double ΔY { get; }
            public double ΔΖ { get; }

            public override double GetVolume() => ΔX * ΔY * ΔΖ;
            public override (double I_1, double I_2, double I_3) GetUnitMmoi()
                => ((ΔY * ΔY + ΔΖ * ΔΖ)/12, (ΔX * ΔX + ΔΖ * ΔΖ)/12, (ΔX * ΔX + ΔY * ΔY)/12);

            public override string ToString() => $"Cuboid({ΔX},{ΔY},{ΔΖ})";
        }
        internal class CylinderShape : Shape
        {
            public CylinderShape(Vector3 position, Quaternion orientation, double length, double radius)
                : base(position, orientation)
            {
                Length = length;
                Radius = radius;
            }
            public double Length { get; set; }
            public double Radius { get; set; }
            public override double GetVolume() => Length * Math.PI * Radius * Radius;
            public override (double I_1, double I_2, double I_3) GetUnitMmoi()
                => (Length * Length/12 + Radius * Radius/4, Length * Length/12 + Radius * Radius/4, Radius * Radius/2);
            public override string ToString() => $"Cylinder({Length},{Radius})";
        }
    }

}

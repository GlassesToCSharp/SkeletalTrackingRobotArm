using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    public class Vector3
    {
        private double x = 0.0;
        private double y = 0.0;
        private double z = 0.0;
        private double magnitude = 0.0;

        public double X
        {
            get { return x; }
            set
            {
                x = value;
                magnitude = Math.Sqrt((x * x) + (y * y) + (z * z));
            }
        }
        public double Y
        {
            get { return y; }
            set
            {
                y = value;
                magnitude = Math.Sqrt((x * x) + (y * y) + (z * z));
            }
        }
        public double Z
        {
            get { return z; }
            set
            {
                z = value;
                magnitude = Math.Sqrt((x * x) + (y * y) + (z * z));
            }
        }

        public double Magnitude
        {
            get
            {
                return (this is Plane ? Double.NaN : magnitude);
            }
        }

        public static Vector3 operator *(Vector3 vec, double c)
        {
            return c * vec;
        }

        public static Vector3 operator *(double c, Vector3 vec)
        {
            Vector3 ret = new Vector3()
            {
                X = vec.X * c,
                Y = vec.Y * c,
                Z = vec.Z * c
            };

            return ret;
        }

        public static Vector3 operator -(Vector3 vec1, Vector3 vec2)
        {
            Vector3 ret = new Vector3()
            {
                X = vec1.X - vec2.X,
                Y = vec1.Y - vec2.Y,
                Z = vec1.Z - vec2.Z
            };

            return ret;
        }

        public double Dot(Vector3 vec)
        {
            double dotProductResult =
                (X * vec.X) +
                (Y * vec.Y) +
                (Z * vec.Z);

            return dotProductResult;
        }

        public Vector3 Cross(Vector3 vec)
        {
            return new Vector3
            {
                X = (Y * vec.Z) - (Z * vec.Y),
                Y = (Z * vec.X) - (X * vec.Z),
                Z = (X * vec.Y) - (Y * vec.X)
            };
        }

        public static double GetAngleBetweenVectors(Vector3 u, Vector3 v)
        {
            // Do dot product
            double dotProductResult = u.Dot(v);

            // Magnitude of u
            double magnitudeOfU = Math.Sqrt((u.X * u.X) + (u.Y * u.Y) + (u.Z * u.Z));

            // Magnitude of v
            double magnitudeOfV = Math.Sqrt((v.X * v.X) + (v.Y * v.Y) + (v.Z * v.Z));

            // arccose of [dot] / (||u|| * ||v||)
            double angle = Math.Acos(dotProductResult / (magnitudeOfU * magnitudeOfV));

            return angle;
        }


    }

    /// <summary>
    /// Gives vector perpendicular to plane.
    /// Should return in the format Xx + Yy + Zz + D = 0
    /// </summary>
    public class Plane : Vector3
    {
        public double D { get; set; } = 0.0;
    }
}

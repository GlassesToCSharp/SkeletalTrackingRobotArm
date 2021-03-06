using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    public class Vector3
    {
        public double X { get; set; } = 0.0;
        public double Y { get; set; } = 0.0;
        public double Z { get; set; } = 0.0;

        public virtual double Magnitude => Math.Sqrt((X * X) + (Y * Y) + (Z * Z));

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

        public CameraSpacePoint toCameraSpacePoint()
        {
            return new CameraSpacePoint()
            {
                X = (float)X,
                Y = (float)Y,
                Z = (float)Z,
            };
        }

        public static double GetAngleBetweenVectors(Vector3 u, Vector3 v, bool inDegrees = false)
        {
            // Do dot product
            double dotProductResult = u.Dot(v);

            // arccose of [dot] / (||u|| * ||v||)
            double angle = Math.Acos(dotProductResult / (u.Magnitude * v.Magnitude));

            if (inDegrees)
            {
                angle = angle * (180 / Math.PI);
            }

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

        override public double Magnitude => Double.NaN;
    }
}

using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    class Constants
    {
        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        public const float InferredZPositionClamp = 0.1f;
    }

    public class Vector3 : IEquatable<Vector3>
    {
        public double X { get; private set; } = 0.0;
        public double Y { get; private set; } = 0.0;
        public double Z { get; private set; } = 0.0;

        public bool IsEmpty => X == 0 && Y == 0 && Z == 0;

        public virtual double Magnitude => Math.Sqrt((X * X) + (Y * Y) + (Z * Z));

        public Vector3(double x = 0.0, double y = 0.0, double z = 0.0)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Vector3 FromPoints(Point3 p, Point3 q, float zPositionclamp = Constants.InferredZPositionClamp)
        {
            double pZ = p.Z;
            double qZ = q.Z;
            if (p.Z < 0)
            {
                pZ = zPositionclamp;
            }
            if (q.Z < 0)
            {
                qZ = zPositionclamp;
            }

            return new Vector3
            {
                X = q.X - p.X,
                Y = q.Y - p.Y,
                Z = qZ - pZ
            };
        }

        public static Vector3 FromVector(Vector3 vector)
        {
            return new Vector3(vector.X, vector.Y, vector.Z);
        }

        #region Operators

        public static bool operator ==(Vector3 thisVector, Vector3 otherVector)
        {
            return thisVector.X == otherVector.X && thisVector.Y == otherVector.Y && thisVector.Z == otherVector.Z;
        }

        public static bool operator !=(Vector3 thisVector, Vector3 otherVector)
        {
            return !(thisVector == otherVector);
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

        public override bool Equals(object obj)
        {
            return obj != null && obj is Vector3 && Equals(obj as Vector3);
        }

        public bool Equals(Vector3 other)
        {
            return this == other;
        }

        public override int GetHashCode()
        {
            unchecked // Overflow is fine, just wrap
            {
                int hash = 17;
                hash = hash * 23 + X.GetHashCode();
                hash = hash * 23 + Y.GetHashCode();
                hash = hash * 23 + Z.GetHashCode();
                return hash;
            }
        }

        #endregion

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

        public static double GetAngleBetweenVectors(Vector3 u, Vector3 v, bool inRadians = false)
        {
            // Do dot product
            double dotProductResult = u.Dot(v);

            // arccose of [dot] / (||u|| * ||v||)
            double angle = Math.Acos(dotProductResult / (u.Magnitude * v.Magnitude));

            if (!inRadians)
            {
                angle *= (180 / Math.PI);
            }

            return angle;
        }

        public override string ToString()
        {
            return String.Format("Vector3[X:{0}, Y:{1}, Z:{2}]", X, Y, Z);
        }
    }
}

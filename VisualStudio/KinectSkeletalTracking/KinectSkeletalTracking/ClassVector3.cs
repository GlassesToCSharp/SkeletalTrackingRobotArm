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
        public double X { get; set; } = 0.0;
        public double Y { get; set; } = 0.0;
        public double Z { get; set; } = 0.0;

        public virtual double Magnitude => Math.Sqrt((X * X) + (Y * Y) + (Z * Z));

        public Vector3(double x = 0.0, double y = 0.0, double z = 0.0)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Vector3 FromPoints(Point3 p, Point3 q, float zPositionclamp = Constants.InferredZPositionClamp)
        {
            if (p.Z < 0)
            {
                p.Z = zPositionclamp;
            }
            if (q.Z < 0)
            {
                q.Z = zPositionclamp;
            }

            return new Vector3
            {
                X = q.X - p.X,
                Y = q.Y - p.Y,
                Z = q.Z - p.Z
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

    /// <summary>
    /// Gives vector perpendicular to plane.
    /// Should return in the format Xx + Yy + Zz + D = 0
    /// </summary>
    public class Plane : Vector3
    {
        public double D { get; set; } = 0.0;

        override public double Magnitude => Double.NaN;

        public static Plane FromPoints(params CameraSpacePoint[] points)
        {
            // minimum 3 points are required
            if (points.Length < 3)
            {
                return null;
            }

            // Only 3 points are needed to make a plane
            Point3 A = Point3.FromCameraSpacePoint(points[0]);
            Point3 B = Point3.FromCameraSpacePoint(points[1]);
            Point3 C = Point3.FromCameraSpacePoint(points[2]);

            Vector3 AB = FromPoints(A, B);
            Vector3 AC = FromPoints(A, C);

            // Do the cross product
            Vector3 normal = AB.Cross(AC);

            // Return the normal with the ammended D value. This is the Plane equation.
            return new Plane
            {
                X = normal.X,
                Y = normal.Y,
                Z = normal.Z,
                D = -((normal.X * (-A.X)) + (normal.Y * (-A.Y)) + (normal.Z * (-A.Z)))
            };
        }


        public static Plane FromVectors(params Vector3[] vectors)
        {
            // minimum 2 vectors are required
            if (vectors.Length < 2)
            {
                return null;
            }

            // Only 2 vectors are needed to make a plane
            Vector3 u = vectors[0];
            Vector3 v = vectors[1];

            // Do the cross product
            Vector3 orthogonal = u.Cross(v);

            // Return the normal with the ammended D value. This is the Plane equation.
            return new Plane
            {
                X = orthogonal.X,
                Y = orthogonal.Y,
                Z = orthogonal.Z,
                D = -((orthogonal.X * (-u.X)) + (orthogonal.Y * (-u.Y)) + (orthogonal.Z * (-u.Z)))
            };
        }
    }
}

using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    public class Plane : IEquatable<Plane>
    {
        public double X { get; set; } = 0.0;
        public double Y { get; set; } = 0.0;
        public double Z { get; set; } = 0.0;
        public double D { get; set; } = 0.0;

        public Plane(double x = 0, double y = 0, double z = 0, double d = 0)
        {
            X = x;
            Y = y;
            Z = z;
            D = d;
        }

        #region Operations

        public static bool operator ==(Plane thisPlane, Plane otherPlane)
        {
            return thisPlane.X == otherPlane.X && thisPlane.Y == otherPlane.Y && thisPlane.Z == otherPlane.Z;
        }

        public static bool operator !=(Plane thisPlane, Plane otherVector)
        {
            return !(thisPlane == otherVector);
        }

        public override bool Equals(object obj)
        {
            return obj != null && obj is Plane && Equals(obj as Plane);
        }

        public bool Equals(Plane other)
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
                hash = hash * 23 + D.GetHashCode();
                return hash;
            }
        }

        #endregion

        public Vector3 AsVector()
        {
            return new Vector3(X, Y, Z);
        }

        public static Plane FromPoints(Point3 a, Point3 b, Point3 c)
        {
            Vector3 AB = Vector3.FromPoints(a, b);
            Vector3 AC = Vector3.FromPoints(a, c);

            // Do the cross product
            Vector3 normal = AB.Cross(AC);

            // Return the normal with the ammended D value. This is the Plane equation.
            return new Plane
            {
                X = normal.X,
                Y = normal.Y,
                Z = normal.Z,
                D = -((normal.X * (-a.X)) + (normal.Y * (-b.Y)) + (normal.Z * (-c.Z)))
            };
        }


        public static Plane FromVectors(Vector3 u, Vector3 v)
        {
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

        public static double GetAngleBetweenPlanes(Plane plane1, Plane plane2, bool inRadians = false)
        {
            Vector3 vector1 = plane1.AsVector();
            Vector3 vector2 = plane2.AsVector();

            return Vector3.GetAngleBetweenVectors(vector1, vector2, inRadians);
        }

        public override string ToString()
        {
            return String.Format("Plane[X:{0}, Y:{1}, Z:{2}, D:{3}]", X, Y, Z, D);
        }
    }
}

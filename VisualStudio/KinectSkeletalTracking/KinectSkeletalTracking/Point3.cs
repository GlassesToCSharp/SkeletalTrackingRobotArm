using Microsoft.Kinect;
using System;

namespace KinectSkeletalTracking
{
    public class Point3 : IEquatable<Point3>
    {
        public double X { get; private set; } = 0.0;
        public double Y { get; private set; } = 0.0;
        public double Z { get; private set; } = 0.0;

        public Point3(double x = 0.0, double y = 0.0, double z = 0.0)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Point3 FromCameraSpacePoint(CameraSpacePoint point)
        {
            if (point == null)
            {
                return new Point3();
            }
            else
            {
                return new Point3(point.X, point.Y, point.Z);
            }
        }

        #region Operations

        public static bool operator ==(Point3 thisPoint, Point3 otherPoint)
        {
            return thisPoint.X == otherPoint.X && thisPoint.Y == otherPoint.Y && thisPoint.Z == otherPoint.Z;
        }

        public static bool operator !=(Point3 thisPoint, Point3 otherPoint)
        {
            return !(thisPoint == otherPoint);
        }

        public static Point3 operator +(Point3 p1, Point3 p2)
        {
            Point3 ret = new Point3()
            {
                X = p1.X + p2.X,
                Y = p1.Y + p2.Y,
                Z = p1.Z + p2.Z
            };

            return ret;
        }

        public static Point3 operator -(Point3 p1, Point3 p2)
        {
            Point3 ret = new Point3()
            {
                X = p1.X - p2.X,
                Y = p1.Y - p2.Y,
                Z = p1.Z - p2.Z
            };

            return ret;
        }

        public override bool Equals(object obj)
        {
            return obj != null && obj is Point3 && Equals(obj as Point3);
        }

        public bool Equals(Point3 other)
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

        public CameraSpacePoint ToCameraSpacePoint()
        {
            return new CameraSpacePoint()
            {
                X = (float)X,
                Y = (float)Y,
                Z = (float)Z,
            };
        }

        /// <summary>
        /// Calculates the shortest distance to a given plane.
        /// </summary>
        /// <param name="plane">The plane to calculate the distance from.</param>
        /// <returns>The distance.</returns>
        public double DistanceFromPlane(Plane plane)
        {
            double distanceFromPlane = plane.AsVector().Dot(X, Y, Z);
            distanceFromPlane += plane.D;
            distanceFromPlane /= plane.AsVector().Magnitude;

            return distanceFromPlane;
        }

        /// <summary>
        /// Calculates the projected point onto a given plane.
        /// </summary>
        /// <param name="plane">The plane to project the point on.</param>
        /// <param name="pointOnPlane">A point on the plane.</param>
        /// <returns>The projected point.</returns>
        public Point3 ProjectedOntoPlane(Plane plane, Point3 pointOnPlane)
        {
            // Steps taken from https://math.stackexchange.com/a/100766
            //double t = (plane.X * pointOnPlane.X - plane.X * X) + (plane.Y * pointOnPlane.Y - plane.Y * Y) + (plane.Z * pointOnPlane.Z - plane.Z * Z);
            //t /= plane.AsVector().Magnitude;

            //return new Point3(X + t * plane.X, Y + t * plane.Y, Z + t * plane.Z);

            // Steps taken from https://math.stackexchange.com/questions/444968/project-a-point-in-3d-on-a-given-plane
            //Vector3 thisToPlane = Vector3.FromPoints(this, pointOnPlane);
            //Vector3 vec = Vector3.FromPoints(new Point3(), this) - (thisToPlane.Dot(plane.AsVector()) * thisToPlane);
            //return new Point3(vec.X, vec.Y, vec.Z);

            double dist = DistanceFromPlane(plane);
            double vecMag = plane.AsVector().Magnitude;
            double factor = dist / vecMag;
            Vector3 movingVector = plane.AsVector() * -(factor);
            return this + new Point3(movingVector.X, movingVector.Y, movingVector.Z);
        }

        /// <summary>
        /// Calculates the projected point onto a given plane.
        /// </summary>
        /// <param name="plane">The plane to project the point on.</param>
        /// <returns>The projection vector.</returns>
        public Vector3 ProjectionVectorToPlane(Plane plane)
        {
            double dist = DistanceFromPlane(plane);
            double vecMag = plane.AsVector().Magnitude;
            double factor = dist / vecMag;
            return plane.AsVector() * -(factor);
        }

        public override string ToString()
        {
            return String.Format("Point3[X:{0}, Y:{1}, Z:{2}]", X, Y, Z);
        }
    }
}

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

        public override string ToString()
        {
            return String.Format("Point3[X:{0}, Y:{1}, Z:{2}]", X, Y, Z);
        }
    }
}

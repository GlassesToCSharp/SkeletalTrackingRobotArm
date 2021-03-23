using KinectSkeletalTracking;
using MatrixDesign;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace KinematicsTests
{
    [TestClass]
    public class ForwardKinematicsTests
    {
        private const double upperArmLength = 1;
        private const double lowerArmLength = 1;
        private static readonly Point3 shoulderPosition = new Point3(10, 10, 10);
        private const double _90Deg = 90 * (Math.PI / 180);

        [TestClass]
        public class RightArm
        {
            [TestClass]
            public class ElbowPosition
            {
                [TestMethod]
                public void Origin()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
                    Point3 point = ForwardKinematics.GetElbowPointRight(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X + upperArmLength,
                        shoulderPosition.Y,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestMethod]
                public void PointingForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
                    Point3 point = ForwardKinematics.GetElbowPointRight(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X,
                        shoulderPosition.Y,
                        shoulderPosition.Z - upperArmLength
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestMethod]
                public void PointingDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
                    Point3 point = ForwardKinematics.GetElbowPointRight(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X,
                        shoulderPosition.Y - upperArmLength,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestMethod]
                public void PointingUp()
                {
                    double shoulderYaw = -_90Deg;
                    double shoulderPitch = _90Deg;
                    Point3 point = ForwardKinematics.GetElbowPointRight(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X,
                        shoulderPosition.Y + upperArmLength,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }
            }

            [TestClass]
            public class WristPosition
            {
                [TestClass]
                public class ElbowSide
                {
                    private const double shoulderYaw = 0;
                    private const double shoulderPitch = 0;

                    [TestMethod]
                    public void Origin()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = 0;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X + upperArmLength + lowerArmLength,
                            shoulderPosition.Y,
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingForward()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = -_90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X + upperArmLength,
                            shoulderPosition.Y,
                            shoulderPosition.Z - lowerArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingUp()
                    {
                        double shoulderRoll = _90Deg;
                        double elbowPitch = -_90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X + upperArmLength,
                            shoulderPosition.Y + lowerArmLength,
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingDown()
                    {
                        double shoulderRoll = -_90Deg;
                        double elbowPitch = -_90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X + upperArmLength,
                            shoulderPosition.Y - lowerArmLength,
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }
                }

                [TestClass]
                public class ElbowDown
                {
                    private const double shoulderYaw = _90Deg;
                    private const double shoulderPitch = _90Deg;

                    [TestMethod]
                    public void PointingDown()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = 0;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y - (upperArmLength + lowerArmLength),
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingForward()
                    {
                        double shoulderRoll = _90Deg;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y - upperArmLength,
                            shoulderPosition.Z - lowerArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingSide()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X - lowerArmLength,
                            shoulderPosition.Y - upperArmLength,
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }
                }

                [TestClass]
                public class ElbowForward
                {
                    private const double shoulderYaw = 0;
                    private const double shoulderPitch = -_90Deg;

                    [TestMethod]
                    public void PointingForward()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = 0;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y,
                            shoulderPosition.Z - (upperArmLength + lowerArmLength)
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingUp()
                    {
                        double shoulderRoll = -_90Deg;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y + lowerArmLength,
                            shoulderPosition.Z - upperArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingSide()
                    {
                        double shoulderRoll = 2 * _90Deg;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X - lowerArmLength,
                            shoulderPosition.Y,
                            shoulderPosition.Z - upperArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }
                }

                private static Point3 GetWristPoint(double shoulderYaw, double shoulderPitch, double shoulderRoll, double elbowPitch)
                {
                    Matrix elbowTranslationMatrix = ForwardKinematics.GetElbowTranslationMatrixRight(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);
                    Point3 point = ForwardKinematics.GetWristPointRight(elbowTranslationMatrix, shoulderRoll, elbowPitch, lowerArmLength);
                    return point;
                }
            }
        }

        [TestClass]
        public class LeftArm
        {
            [TestClass]
            public class ElbowPosition
            {
                [TestMethod]
                public void Origin()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
                    Point3 point = ForwardKinematics.GetElbowPointLeft(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X - upperArmLength,
                        shoulderPosition.Y,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestMethod]
                public void PointingDown()
                {
                    double shoulderYaw = -_90Deg;
                    double shoulderPitch = _90Deg;
                    Point3 point = ForwardKinematics.GetElbowPointLeft(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X,
                        shoulderPosition.Y - upperArmLength,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestMethod]
                public void PointingForward()
                {
                    // Yaw will go from 90 to 270, making 180 the mid point.
                    double shoulderYaw = 2 * _90Deg;
                    double shoulderPitch = -_90Deg;
                    Point3 point = ForwardKinematics.GetElbowPointLeft(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X,
                        shoulderPosition.Y,
                        shoulderPosition.Z - upperArmLength
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestMethod]
                public void PointingUp()
                {
                    double shoulderYaw = -_90Deg;
                    double shoulderPitch = -_90Deg;
                    Point3 point = ForwardKinematics.GetElbowPointLeft(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X,
                        shoulderPosition.Y + upperArmLength,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }
            }

            [TestClass]
            public class WristPosition
            {

                [TestMethod]
                public void Origin()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
                    double shoulderRoll = 0;
                    double elbowPitch = 0;
                    Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Point3 expectedPoint = new Point3(
                        shoulderPosition.X - upperArmLength - lowerArmLength,
                        shoulderPosition.Y,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestClass]
                public class ElbowPointingSide
                {
                    private static double shoulderYaw = 0;
                    private static double shoulderPitch = 0;

                    [TestMethod]
                    public void PointingDown()
                    {
                        double shoulderRoll = -_90Deg;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X - upperArmLength,
                            shoulderPosition.Y - lowerArmLength,
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingForward()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X - upperArmLength,
                            shoulderPosition.Y,
                            shoulderPosition.Z - lowerArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingUp()
                    {
                        double shoulderRoll = _90Deg;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X - upperArmLength,
                            shoulderPosition.Y + lowerArmLength,
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }
                }

                [TestClass]
                public class ElbowPointingDown
                {
                    private static double shoulderYaw = -_90Deg;
                    private static double shoulderPitch = _90Deg;

                    [TestMethod]
                    public void PointingDown()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = 0;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y - (upperArmLength + lowerArmLength),
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingForward()
                    {
                        double shoulderRoll = _90Deg;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y - upperArmLength,
                            shoulderPosition.Z - lowerArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingSide()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X + lowerArmLength,
                            shoulderPosition.Y - upperArmLength,
                            shoulderPosition.Z
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }
                }

                [TestClass]
                public class ElbowPointingForward
                {
                    private static double shoulderYaw = 0;
                    private static double shoulderPitch = _90Deg;

                    [TestMethod]
                    public void PointingForward()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = 0;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y,
                            shoulderPosition.Z - (upperArmLength + lowerArmLength)
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingSide()
                    {
                        double shoulderRoll = 0;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X + lowerArmLength,
                            shoulderPosition.Y,
                            shoulderPosition.Z - upperArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }

                    [TestMethod]
                    public void PointingUp()
                    {
                        double shoulderRoll = _90Deg;
                        double elbowPitch = _90Deg;
                        Point3 point = GetWristPoint(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                        Point3 expectedPoint = new Point3(
                            shoulderPosition.X,
                            shoulderPosition.Y + lowerArmLength,
                            shoulderPosition.Z - upperArmLength
                        );

                        Assert.AreEqual(expectedPoint, point);
                    }
                }

                private static Point3 GetWristPoint(double shoulderYaw, double shoulderPitch, double shoulderRoll, double elbowPitch)
                {
                    Matrix elbowTranslationMatrix = ForwardKinematics.GetElbowTranslationMatrixLeft(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);
                    Point3 point = ForwardKinematics.GetWristPointLeft(elbowTranslationMatrix, shoulderRoll, elbowPitch, lowerArmLength);
                    return point;
                }
            }
        }
    }
}

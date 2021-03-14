using KinectSkeletalTracking;
using MatrixDesign;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace KinematicsTests
{
    [TestClass]
    public class BodyAngles
    {
        const double angleTolerance = 0.000000001;
        private static readonly Point3 neck = new Point3(9.5, 10, 10);
        private static readonly Point3 spine = new Point3(9.5, 9, 10);
        private static readonly Point3 shoulderL = new Point3(9, 10, 10);
        private static readonly Point3 shoulderR = new Point3(10, 10, 10);
        private static readonly Vector3 neckToSpine = Vector3.FromPoints(neck, spine);
        private static readonly Vector3 crossShoulder = Vector3.FromPoints(shoulderL, shoulderR);

        [TestClass]
        public class ShoulderYaw
        {
            [TestMethod]
            public void Inline()
            {
                Point3 elbow = new Point3(11, 10, 10);
                double yaw = Kinematics.GetShoulderYaw(neck, spine, shoulderL, shoulderR, elbow);

                Assert.AreEqual(0, yaw, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingForward()
            {
                Point3 elbow = new Point3(10, 10, 9);
                double yaw = Kinematics.GetShoulderYaw(neck, spine, shoulderL, shoulderR, elbow);

                Assert.AreEqual(0.0, yaw, angleTolerance);
            }
        }

        [TestClass]
        public class ShoulderPitch
        {
            [TestMethod]
            public void Inline()
            {
                Point3 elbow = new Point3(11, 10, 10);
                double pitch = Kinematics.GetShoulderPitch(shoulderL, shoulderR, elbow);

                Assert.AreEqual(90, pitch, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingDown()
            {
                Point3 elbow = new Point3(10, 9, 10);
                double pitch = Kinematics.GetShoulderPitch(shoulderL, shoulderR, elbow);

                Assert.AreEqual(0, pitch, angleTolerance);
            }
        }

        [TestClass]
        public class ShoulderRoll
        {
            private static readonly Vector3 shoulderToElbow = new Vector3(-1, 0, 0);

            private double CompensatedRoll(double roll, bool inRadians=false)
            {
                if (roll >= 0)
                {
                    return roll;
                }

                double compensation = inRadians ? Math.PI * 2 : 360;
                return roll + compensation;
            }

            [TestMethod]
            public void Inline()
            {
                Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                double roll = Kinematics.GetShoulderRoll(neckToSpine, crossShoulder, shoulderToElbow, elbowToWrist);

                Assert.AreEqual(270, CompensatedRoll(roll));
            }

            [TestMethod]
            public void WristPointingForward()
            {
                Vector3 elbowToWrist = new Vector3(0, 0, -1);
                double roll = Kinematics.GetShoulderRoll(neckToSpine, crossShoulder, shoulderToElbow, elbowToWrist);

                Assert.AreEqual(180, CompensatedRoll(roll), angleTolerance);
            }

            [TestMethod]
            public void WristPointingUp()
            {
                Vector3 elbowToWrist = new Vector3(0, 1, 0);
                double roll = Kinematics.GetShoulderRoll(neckToSpine, crossShoulder, shoulderToElbow, elbowToWrist);

                Assert.AreEqual(270, CompensatedRoll(roll), angleTolerance);
            }

            [TestMethod]
            public void WristPointingDown()
            {
                Vector3 elbowToWrist = new Vector3(0, -1, 0);
                double roll = Kinematics.GetShoulderRoll(neckToSpine, crossShoulder, shoulderToElbow, elbowToWrist);

                Assert.AreEqual(90, CompensatedRoll(roll), angleTolerance);
            }
        }

        [TestClass]
        public class ForwardKinematics
        {
            private const double upperArmLength = 1;
            private const double lowerArmLength = 1;
            private static readonly Point3 shoulderPosition = new Point3(10, 10, 10);
            private const double _90Deg = 90 * (Math.PI / 180);

            [TestClass]
            public class ElbowPosition
            {
                [TestMethod]
                public void Origin()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
                    Point3 point = Kinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                    Point3 point = Kinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                    Point3 point = Kinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                    Point3 point = Kinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                        shoulderPosition.X + upperArmLength + lowerArmLength,
                        shoulderPosition.Y,
                        shoulderPosition.Z
                    );

                    Assert.AreEqual(expectedPoint, point);
                }

                [TestMethod]
                public void PointingForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
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
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
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
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
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

                [TestMethod]
                public void PointingDownElbowDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
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
                public void PointingForwardElbowForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
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
                public void PointingForwardElbowDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
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
                public void PointingSideElbowDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
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

                [TestMethod]
                public void PointingUpElbowForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
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
                public void PointingSideElbowForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
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

                private Point3 GetWristPoint(double shoulderYaw, double shoulderPitch, double shoulderRoll, double elbowPitch)
                {
                    Matrix elbowTranslationMatrix = Kinematics.GetElbowTranslationMatrix(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);
                    Point3 point = Kinematics.GetWristPoint(elbowTranslationMatrix, shoulderRoll, elbowPitch, lowerArmLength);
                    return point;
                }
            }
        }
    }
}

using KinectSkeletalTracking;
using MatrixDesign;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;

namespace KinematicsTests
{
    [TestClass]
    public class InverseKinematicsTests
    {
        const double angleTolerance = 0.000000001;
        private static readonly Point3 neck = new Point3(9, 12, 10);
        private static readonly Point3 spine = new Point3(9, 8, 10);
        private static readonly Point3 shoulderL = new Point3(8, 10, 10);
        private static readonly Point3 shoulderR = new Point3(10, 10, 10);
        private static readonly Vector3 neckToSpine = Vector3.FromPoints(neck, spine);
        private static readonly Vector3 shoulderL2R = Vector3.FromPoints(shoulderL, shoulderR);

        [TestClass]
        public class ShoulderYaw
        {
            [TestMethod]
            public void Inline()
            {
                Vector3 shoulderToElbow = new Vector3(1, 0, 0);
                double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderL2R, shoulderToElbow);

                Assert.AreEqual(0, yaw, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingDown()
            {
                Vector3 shoulderToElbow = new Vector3(0, -1, 0);
                double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderL2R, shoulderToElbow);

                Assert.AreEqual(-90, yaw, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingForward()
            {
                Vector3 shoulderToElbow = new Vector3(0, 0, -1);
                double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderL2R, shoulderToElbow);

                Assert.AreEqual(0, yaw, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingUp()
            {
                Vector3 shoulderToElbow = new Vector3(0, 1, 0);
                double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderL2R, shoulderToElbow);

                Assert.AreEqual(90, yaw, angleTolerance);
            }
        }

        [TestClass]
        public class ShoulderPitch
        {
            [TestMethod]
            public void Inline()
            {
                Vector3 shoulderToElbow = new Vector3(1, 0, 0);
                double pitch = InverseKinematics.GetShoulderPitch(shoulderL2R, shoulderToElbow);

                Assert.AreEqual(0, pitch, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingDown()
            {
                Vector3 shoulderToElbow = new Vector3(0, -1, 0);
                double pitch = InverseKinematics.GetShoulderPitch(shoulderL2R, shoulderToElbow);

                Assert.AreEqual(-90, pitch, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingForward()
            {
                Vector3 shoulderToElbow = new Vector3(0, 0, -1);
                double pitch = InverseKinematics.GetShoulderPitch(shoulderL2R, shoulderToElbow);

                Assert.AreEqual(-90, pitch, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingUp()
            {
                Vector3 shoulderToElbow = new Vector3(0, 1, 0);
                double pitch = InverseKinematics.GetShoulderPitch(shoulderL2R, shoulderToElbow);

                Assert.AreEqual(-90, pitch, angleTolerance);
            }
        }

        [TestClass]
        public class ShoulderRoll
        {
            [TestClass]
            public class ElbowPointingSide
            {
                private static readonly Vector3 shoulderToElbow = new Vector3(1, 0, 0);
                private static readonly double forwardFacingRatio = 0;

                [TestMethod]
                public void Inline()
                {
                    Vector3 elbowToWrist = new Vector3(1, 0, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(0, roll);
                }

                [TestMethod]
                public void WristPointingDown()
                {
                    Vector3 elbowToWrist = new Vector3(0, -1, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(-90, roll, angleTolerance);
                }

                [TestMethod]
                public void WristPointingForward()
                {
                    Vector3 elbowToWrist = new Vector3(0, 0, -1);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(0, roll, angleTolerance);
                }

                [TestMethod]
                public void WristPointingUp()
                {
                    Vector3 elbowToWrist = new Vector3(0, 1, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(90, roll, angleTolerance);
                }
            }

            [TestClass]
            public class ElbowPointingDown
            {
                private static readonly Vector3 shoulderToElbow = new Vector3(0, -1, 0);
                private static readonly double forwardFacingRatio = 0;

                [TestMethod]
                public void Inline()
                {
                    Vector3 elbowToWrist = new Vector3(0, -1, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(0, roll);
                }

                [TestMethod]
                public void WristPointingForward()
                {
                    Vector3 elbowToWrist = new Vector3(0, 0, -1);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(0, roll, angleTolerance);
                }

                [TestMethod]
                public void WristPointingInside()
                {
                    Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(-90, roll, angleTolerance);
                }

                [TestMethod]
                public void WristPointingOutside()
                {
                    Vector3 elbowToWrist = new Vector3(1, 0, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(90, roll, angleTolerance);
                }
            }

            [TestClass]
            public class ElbowPointingForward
            {
                private static readonly Vector3 shoulderToElbow = new Vector3(0, 0, -1);
                private static readonly double forwardFacingRatio = 1;

                [TestMethod]
                public void Inline()
                {
                    Vector3 elbowToWrist = new Vector3(0, 0, -1);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(0, roll);
                }

                [TestMethod]
                public void WristPointingInside()
                {
                    Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(0, roll, angleTolerance);
                }

                [TestMethod]
                public void WristPointingUp()
                {
                    Vector3 elbowToWrist = new Vector3(0, 1, 0);
                    double roll = InverseKinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                    Assert.AreEqual(90, roll, angleTolerance);
                }
            }
        }

        [TestClass]
        public class ElbowPitch
        {
            private static readonly Vector3 shoulderToElbow = new Vector3(1, 0, 0);

            [TestMethod]
            public void Inline()
            {
                Vector3 elbowToWrist = new Vector3(1, 0, 0);
                double angle = InverseKinematics.GetElbowAngle(shoulderToElbow, elbowToWrist);

                Assert.AreEqual(0, angle, angleTolerance);
            }

            [TestMethod]
            public void WristPointingDown()
            {
                Vector3 elbowToWrist = new Vector3(0, 0, -1);
                double angle = InverseKinematics.GetElbowAngle(shoulderToElbow, elbowToWrist);

                Assert.AreEqual(-90, angle, angleTolerance);
            }

            [TestMethod]
            public void WristPointingForward()
            {
                Vector3 elbowToWrist = new Vector3(0, 0, -1);
                double angle = InverseKinematics.GetElbowAngle(shoulderToElbow, elbowToWrist);

                Assert.AreEqual(-90, angle, angleTolerance);
            }

            [TestMethod]
            public void WristPointingUp()
            {
                Vector3 elbowToWrist = new Vector3(0, 0, -1);
                double angle = InverseKinematics.GetElbowAngle(shoulderToElbow, elbowToWrist);

                Assert.AreEqual(-90, angle, angleTolerance);
            }
        }

        [TestClass]
        public class ForwardKinematicsTests
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
                    Point3 point = ForwardKinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                    Point3 point = ForwardKinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                    Point3 point = ForwardKinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                    Point3 point = ForwardKinematics.GetElbowPoint(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

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
                    Matrix elbowTranslationMatrix = ForwardKinematics.GetElbowTranslationMatrix(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);
                    Point3 point = ForwardKinematics.GetWristPoint(elbowTranslationMatrix, shoulderRoll, elbowPitch, lowerArmLength);
                    return point;
                }
            }
        }
    }
}

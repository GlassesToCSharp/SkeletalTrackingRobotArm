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
        private static readonly Vector3 spine = new Vector3(0, -1, 0);
        private static readonly Vector3 crossShoulder = new Vector3(-2, 0, 0);
        private static readonly Plane body = Plane.FromVectors(spine, crossShoulder);

        [TestClass]
        public class ShoulderYaw
        {
            [TestMethod]
            public void Inline()
            {
                Vector3 shoulderToElbow = new Vector3(-1, 0, 0);
                double yaw = Kinematics.GetShoulderYaw(spine, crossShoulder, shoulderToElbow);

                Assert.IsTrue(Double.IsNaN(yaw));
            }

            [TestMethod]
            public void ElbowPointingForward()
            {
                Vector3 shoulderToElbow = new Vector3(0, 0, -1);
                double yaw = Kinematics.GetShoulderYaw(spine, crossShoulder, shoulderToElbow);

                Assert.AreEqual(90, yaw, angleTolerance);
            }
        }

        [TestClass]
        public class ShoulderPitch
        {
            [TestMethod]
            public void Inline()
            {
                Vector3 shoulderToElbow = new Vector3(-1, 0, 0);
                double pitch = Kinematics.GetShoulderPitch(crossShoulder, shoulderToElbow);

                Assert.AreEqual(0, pitch, angleTolerance);
            }

            [TestMethod]
            public void ElbowPointingDown()
            {
                Vector3 shoulderToElbow = new Vector3(0, -1, 0);
                double pitch = Kinematics.GetShoulderPitch(crossShoulder, shoulderToElbow);

                Assert.AreEqual(90, pitch, angleTolerance);
            }
        }

        [TestClass]
        public class ShoulderRoll
        {
            private static readonly Vector3 shoulderToElbow = new Vector3(-1, 0, 0);

            [TestMethod]
            public void Inline()
            {
                Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                double roll = Kinematics.GetShoulderRoll(body, shoulderToElbow, elbowToWrist);

                Assert.IsTrue(Double.IsNaN(roll));
            }

            [TestMethod]
            public void WristPointingForward()
            {
                Vector3 elbowToWrist = new Vector3(0, 0, -1);
                double roll = Kinematics.GetShoulderRoll(body, shoulderToElbow, elbowToWrist);

                Assert.AreEqual(0, roll, angleTolerance);
            }

            [TestMethod]
            public void WristPointingUp()
            {
                Vector3 elbowToWrist = new Vector3(0, 1, 0);
                double roll = Kinematics.GetShoulderRoll(body, shoulderToElbow, elbowToWrist);

                Assert.AreEqual(90, roll, angleTolerance);
            }

            [TestMethod]
            public void WristPointingDown()
            {
                Vector3 elbowToWrist = new Vector3(0, 1, 0);
                double roll = Kinematics.GetShoulderRoll(body, shoulderToElbow, elbowToWrist);

                Assert.AreEqual(90, roll, angleTolerance);
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
                    Vector3 vector = Kinematics.GetElbowVector(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X + upperArmLength,
                        Y = shoulderPosition.Y,
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
                    Vector3 vector = Kinematics.GetElbowVector(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X,
                        Y = shoulderPosition.Y,
                        Z = shoulderPosition.Z - upperArmLength
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
                    Vector3 vector = Kinematics.GetElbowVector(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X,
                        Y = shoulderPosition.Y - upperArmLength,
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingUp()
                {
                    double shoulderYaw = -_90Deg;
                    double shoulderPitch = _90Deg;
                    Vector3 vector = Kinematics.GetElbowVector(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X,
                        Y = shoulderPosition.Y + upperArmLength,
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
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
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X + upperArmLength + lowerArmLength,
                        Y = shoulderPosition.Y,
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
                    double shoulderRoll = 0;
                    double elbowPitch = -_90Deg;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X + upperArmLength,
                        Y = shoulderPosition.Y,
                        Z = shoulderPosition.Z - lowerArmLength
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingUp()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
                    double shoulderRoll = _90Deg;
                    double elbowPitch = -_90Deg;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X + upperArmLength,
                        Y = shoulderPosition.Y + lowerArmLength,
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingDown()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = 0;
                    double shoulderRoll = -_90Deg;
                    double elbowPitch = -_90Deg;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X + upperArmLength,
                        Y = shoulderPosition.Y - lowerArmLength,
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingDownElbowDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
                    double shoulderRoll = 0;
                    double elbowPitch = 0;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X,
                        Y = shoulderPosition.Y - (upperArmLength + lowerArmLength),
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingForwardElbowForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
                    double shoulderRoll = 0;
                    double elbowPitch = 0;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X,
                        Y = shoulderPosition.Y,
                        Z = shoulderPosition.Z - (upperArmLength + lowerArmLength)
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingForwardElbowDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
                    double shoulderRoll = _90Deg;
                    double elbowPitch = _90Deg;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X,
                        Y = shoulderPosition.Y - upperArmLength,
                        Z = shoulderPosition.Z - lowerArmLength
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingSideElbowDown()
                {
                    double shoulderYaw = _90Deg;
                    double shoulderPitch = _90Deg;
                    double shoulderRoll = 0;
                    double elbowPitch = _90Deg;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X - lowerArmLength,
                        Y = shoulderPosition.Y - upperArmLength,
                        Z = shoulderPosition.Z
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingUpElbowForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
                    double shoulderRoll = -_90Deg;
                    double elbowPitch = _90Deg;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X,
                        Y = shoulderPosition.Y + lowerArmLength,
                        Z = shoulderPosition.Z - upperArmLength
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                [TestMethod]
                public void PointingSideElbowForward()
                {
                    double shoulderYaw = 0;
                    double shoulderPitch = -_90Deg;
                    double shoulderRoll = 2 * _90Deg;
                    double elbowPitch = _90Deg;
                    Vector3 vector = GetWristVector(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch);

                    Vector3 expectedVector = new Vector3()
                    {
                        X = shoulderPosition.X - lowerArmLength,
                        Y = shoulderPosition.Y,
                        Z = shoulderPosition.Z - upperArmLength
                    };

                    Assert.AreEqual(expectedVector, vector);
                }

                private Vector3 GetWristVector(double shoulderYaw, double shoulderPitch, double shoulderRoll, double elbowPitch)
                {
                    Matrix elbowTranslationMatrix = Kinematics.GetElbowTranslationMatrix(shoulderPosition, upperArmLength, shoulderYaw, shoulderPitch);
                    Vector3 vector = Kinematics.GetWristVector(elbowTranslationMatrix, shoulderRoll, elbowPitch, lowerArmLength);
                    return vector;
                }
            }
        }
    }
}

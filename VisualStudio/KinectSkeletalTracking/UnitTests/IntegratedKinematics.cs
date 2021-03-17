using Microsoft.VisualStudio.TestTools.UnitTesting;
using KinectSkeletalTracking;
using System;

namespace KinematicsTests
{
    /// <summary>
    /// These tests look at testing the expected end position of the joints and
    /// the projected points. Each test follows the same structure: using the
    /// given elbow and wrist end position, calculate the angles of each joint
    /// (Inverse Kinematics) and implement them to get the projected end points
    /// (Forward Kinematics). If the projected elbow and wrist points match the
    /// given elbow and wrist points, both the inverse and forward kinematics
    /// equations work. The test passes.
    /// </summary>

    [TestClass]
    public class IntegratedKinematics
    {
        private static readonly Point3 neck = new Point3(9, 12, 10);
        private static readonly Point3 spine = new Point3(9, 8, 10);
        private static readonly Point3 shoulderL = new Point3(8, 10, 10);
        private static readonly Point3 shoulderR = new Point3(10, 10, 10);

        private static readonly Vector3 neckToSpine = Vector3.FromPoints(neck, spine);
        private static readonly Vector3 shoulderL2R = Vector3.FromPoints(shoulderL, shoulderR);

        [TestMethod]
        public void Origin()
        {
            Point3 elbow = new Point3(11, 10, 10);
            Point3 wrist = new Point3(12, 10, 10);

            AssertProjectionsMatchPoints(elbow, wrist);
        }

        [TestClass]
        public class ElbowPointingSide
        {
            private static readonly Point3 elbow = new Point3(11, 10, 10);

            [TestMethod]
            public void WristPointingDown()
            {
                Point3 wrist = new Point3(11, 9, 10);

                AssertProjectionsMatchPoints(elbow, wrist);
            }

            [TestMethod]
            public void WristPointingForward()
            {
                Point3 wrist = new Point3(11, 10, 9);

                AssertProjectionsMatchPoints(elbow, wrist);
            }

            [TestMethod]
            public void WristPointingUp()
            {
                Point3 wrist = new Point3(11, 11, 10);

                AssertProjectionsMatchPoints(elbow, wrist);
            }
        }

        [TestClass]
        public class ElbowPointingDown
        {
            private static readonly Point3 elbow = new Point3(10, 9, 10);

            [TestMethod]
            public void WristPointingForward()
            {
                Point3 wrist = new Point3(10, 9, 9);

                AssertProjectionsMatchPoints(elbow, wrist);
            }

            [TestMethod]
            public void WristPointingInwards()
            {
                Point3 wrist = new Point3(9, 9, 10);

                AssertProjectionsMatchPoints(elbow, wrist);
            }
        }

        [TestClass]
        public class ElbowPointingForward
        {
            public static readonly Point3 elbow = new Point3(10, 10, 9);

            [TestMethod]
            public void WristPointingForward()
            {
                Point3 wrist = new Point3(10, 10, 8);

                AssertProjectionsMatchPoints(elbow, wrist);
            }

            [TestMethod]
            public void WristPointingUp()
            {
                Point3 wrist = new Point3(10, 11, 9);

                AssertProjectionsMatchPoints(elbow, wrist);
            }

            [TestMethod]
            public void WristPointingInwards()
            {
                Point3 wrist = new Point3(9, 10, 9);

                AssertProjectionsMatchPoints(elbow, wrist);
            }
        }

        private static void AssertProjectionsMatchPoints(Point3 elbow, Point3 wrist)
        {
            // Make sure the lengths of each segment are the expected lengths.
            Assert.AreEqual(1, Vector3.FromPoints(shoulderR, elbow).Magnitude);
            Assert.AreEqual(1, Vector3.FromPoints(elbow, wrist).Magnitude);

            Projections projections = Projections.FromPoints(elbow, wrist, Plane.FromPoints(shoulderR, shoulderL, spine));

            Assert.AreEqual(elbow, projections.Elbow);
            Assert.AreEqual(wrist, projections.Wrist);
        }

        private class Projections
        {
            public Point3 Elbow { get; private set; }
            public Point3 Wrist { get; private set; }

            public Projections(Point3 elbow, Point3 wrist)
            {
                Elbow = elbow;
                Wrist = wrist;
            }

            public static Projections FromPoints(Point3 elbow, Point3 wrist, Plane body)
            {
                Vector3 shoulderToElbow = Vector3.FromPoints(shoulderR, elbow);
                Vector3 elbowToWrist = Vector3.FromPoints(elbow, wrist);
                double elbowDistanceFromBody = body.AsVector().Dot(elbow.X, elbow.Y, elbow.Z);
                elbowDistanceFromBody += body.D;
                elbowDistanceFromBody /= body.AsVector().Magnitude;

                double shoulderYaw = Kinematics.GetShoulderYaw(neckToSpine, shoulderL2R, shoulderToElbow, inRadians: true);
                double shoulderPitch = Kinematics.GetShoulderPitch(shoulderL2R, shoulderToElbow, inRadians: true);
                double forwardFacingRatio = Math.Abs(elbowDistanceFromBody / shoulderToElbow.Magnitude);
                double shoulderRoll = Kinematics.GetShoulderRoll(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio, inRadians: true);
                double elbowPitch = Kinematics.GetElbowAngle(shoulderToElbow, elbowToWrist, inRadians: true);

                Point3 elbowProjection = Kinematics.GetElbowPoint(shoulderR, shoulderToElbow.Magnitude, shoulderYaw, shoulderPitch);
                MatrixDesign.Matrix elbowMatrix = Kinematics.GetElbowTranslationMatrix(shoulderR, shoulderToElbow.Magnitude, shoulderYaw, shoulderPitch);
                Point3 wristProjection = Kinematics.GetWristPoint(elbowMatrix, shoulderRoll - shoulderYaw, elbowPitch, elbowToWrist.Magnitude);

                return new Projections(elbowProjection, wristProjection);
            }
        }
    }
}

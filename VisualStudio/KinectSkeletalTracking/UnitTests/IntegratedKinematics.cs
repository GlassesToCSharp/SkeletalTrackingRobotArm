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

            InverseKinematics ik = InverseKinematics.GetInverseKinematics(neck, spine, shoulderL, shoulderR, elbow, wrist, inRadians: true);
            ForwardKinematics fk = ForwardKinematics.GetForwardKinematics(shoulderR, ik);

            Assert.AreEqual(elbow, fk.Elbow);
            Assert.AreEqual(wrist, fk.Wrist);
        }
    }
}

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

        [TestClass]
        public class RightArm
        {
            [TestMethod]
            public void Origin()
            {
                Point3 elbow = new Point3(11, 10, 10);
                Point3 wrist = new Point3(12, 10, 10);

                AssertProjectionsMatchPointsRight(elbow, wrist);
            }

            [TestClass]
            public class ElbowPointingSide
            {
                private static readonly Point3 elbow = new Point3(11, 10, 10);

                [TestMethod]
                public void WristPointingDown()
                {
                    Point3 wrist = new Point3(11, 9, 10);

                    AssertProjectionsMatchPointsRight(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingForward()
                {
                    Point3 wrist = new Point3(11, 10, 9);

                    AssertProjectionsMatchPointsRight(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingUp()
                {
                    Point3 wrist = new Point3(11, 11, 10);

                    AssertProjectionsMatchPointsRight(elbow, wrist);
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

                    AssertProjectionsMatchPointsRight(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingInwards()
                {
                    Point3 wrist = new Point3(9, 9, 10);

                    AssertProjectionsMatchPointsRight(elbow, wrist);
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

                    AssertProjectionsMatchPointsRight(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingUp()
                {
                    Point3 wrist = new Point3(10, 11, 9);

                    AssertProjectionsMatchPointsRight(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingInwards()
                {
                    Point3 wrist = new Point3(9, 10, 9);

                    AssertProjectionsMatchPointsRight(elbow, wrist);
                }
            }
        }

        [TestClass]
        public class LeftArm
        {
            [TestClass]
            public class ElbowPointingSide
            {
                private static readonly Point3 elbow = new Point3(7, 10, 10);

                [TestMethod]
                public void Inline()
                {
                    Point3 wrist = new Point3(6, 10, 10);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingDown()
                {
                    Point3 wrist = new Point3(7, 9, 10);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingForward()
                {
                    Point3 wrist = new Point3(7, 10, 9);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingUp()
                {
                    Point3 wrist = new Point3(7, 11, 10);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }
            }

            [TestClass]
            public class ElbowPointingDown
            {
                private static readonly Point3 elbow = new Point3(8, 9, 10);

                [TestMethod]
                public void Inline()
                {
                    Point3 wrist = new Point3(8, 8, 10);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingForward()//roll + 90
                {
                    Point3 wrist = new Point3(8, 9, 9);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingInwards()// roll 0
                {
                    Point3 wrist = new Point3(9, 9, 10);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }
            }

            [TestClass]
            public class ElbowPointingForward
            {
                public static readonly Point3 elbow = new Point3(8, 10, 9);

                [TestMethod]
                public void Inline()
                {
                    Point3 wrist = new Point3(8, 10, 8);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingForward()
                {
                    Point3 wrist = new Point3(8, 10, 8);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingUp()
                {
                    Point3 wrist = new Point3(8, 11, 9);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }

                [TestMethod]
                public void WristPointingInwards()
                {
                    Point3 wrist = new Point3(9, 10, 9);

                    AssertProjectionsMatchPointsLeft(elbow, wrist);
                }
            }
        }

        private static void AssertProjectionsMatchPointsRight(Point3 elbow, Point3 wrist)
        {
            // Make sure the lengths of each segment are the expected lengths.
            Assert.AreEqual(1, Vector3.FromPoints(shoulderR, elbow).Magnitude);
            Assert.AreEqual(1, Vector3.FromPoints(elbow, wrist).Magnitude);

            InverseKinematics ik = InverseKinematics.GetInverseKinematicsRight(neck, spine, shoulderL, shoulderR, elbow, wrist, inRadians: true);
            ForwardKinematics fk = ForwardKinematics.GetForwardKinematicsRight(shoulderR, ik);

            Assert.AreEqual(elbow, fk.Elbow);
            Assert.AreEqual(wrist, fk.Wrist);
        }

        private static void AssertProjectionsMatchPointsLeft(Point3 elbow, Point3 wrist)
        {
            // Make sure the lengths of each segment are the expected lengths.
            Assert.AreEqual(1, Vector3.FromPoints(shoulderL, elbow).Magnitude);
            Assert.AreEqual(1, Vector3.FromPoints(elbow, wrist).Magnitude);

            InverseKinematics ik = InverseKinematics.GetInverseKinematicsLeft(neck, spine, shoulderL, shoulderR, elbow, wrist, inRadians: true);
            ForwardKinematics fk = ForwardKinematics.GetForwardKinematicsLeft(shoulderL, ik);

            Assert.AreEqual(elbow, fk.Elbow);
            Assert.AreEqual(wrist, fk.Wrist);
        }
    }
}

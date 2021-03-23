using KinectSkeletalTracking;
using Microsoft.VisualStudio.TestTools.UnitTesting;

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
        private static readonly Vector3 shoulderR2L = Vector3.FromPoints(shoulderR, shoulderL);

        [TestClass]
        public class RightArm
        {

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
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll);
                    }

                    [TestMethod]
                    public void WristPointingDown()
                    {
                        Vector3 elbowToWrist = new Vector3(0, -1, 0);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(-90, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingForward()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 0, -1);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingUp()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 1, 0);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

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
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll);
                    }

                    [TestMethod]
                    public void WristPointingForward()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 0, -1);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingInside()
                    {
                        Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(-90, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingOutside()
                    {
                        Vector3 elbowToWrist = new Vector3(1, 0, 0);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

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
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll);
                    }

                    [TestMethod]
                    public void WristPointingInside()
                    {
                        Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingUp()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 1, 0);
                        double roll = InverseKinematics.GetShoulderRollRight(neckToSpine, shoulderL2R, shoulderToElbow, elbowToWrist, forwardFacingRatio);

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
                    Vector3 elbowToWrist = new Vector3(0, 1, 0);
                    double angle = InverseKinematics.GetElbowAngle(shoulderToElbow, elbowToWrist);

                    Assert.AreEqual(-90, angle, angleTolerance);
                }
            }
        }


        [TestClass]
        public class LeftArm
        {
            [TestClass]
            public class ShoulderYaw
            {
                [TestMethod]
                public void Inline()
                {
                    Vector3 shoulderToElbow = new Vector3(-1, 0, 0);
                    double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderR2L, shoulderToElbow);

                    Assert.AreEqual(0, yaw, angleTolerance);
                }

                [TestMethod]
                public void ElbowPointingDown()
                {
                    Vector3 shoulderToElbow = new Vector3(0, -1, 0);
                    double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderR2L, shoulderToElbow);

                    Assert.AreEqual(-90, yaw, angleTolerance);
                }

                [TestMethod]
                public void ElbowPointingForward()
                {
                    Vector3 shoulderToElbow = new Vector3(0, 0, -1);
                    double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderR2L, shoulderToElbow);

                    Assert.AreEqual(0, yaw, angleTolerance);
                }

                [TestMethod]
                public void ElbowPointingUp()
                {
                    Vector3 shoulderToElbow = new Vector3(0, 1, 0);
                    double yaw = InverseKinematics.GetShoulderYaw(neckToSpine, shoulderR2L, shoulderToElbow);

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
                    double pitch = InverseKinematics.GetShoulderPitch(shoulderR2L, shoulderToElbow);

                    Assert.AreEqual(0, pitch, angleTolerance);
                }

                [TestMethod]
                public void ElbowPointingDown()
                {
                    Vector3 shoulderToElbow = new Vector3(0, -1, 0);
                    double pitch = InverseKinematics.GetShoulderPitch(shoulderR2L, shoulderToElbow);

                    Assert.AreEqual(-90, pitch, angleTolerance);
                }

                [TestMethod]
                public void ElbowPointingForward()
                {
                    Vector3 shoulderToElbow = new Vector3(0, 0, -1);
                    double pitch = InverseKinematics.GetShoulderPitch(shoulderR2L, shoulderToElbow);

                    Assert.AreEqual(-90, pitch, angleTolerance);
                }

                [TestMethod]
                public void ElbowPointingUp()
                {
                    Vector3 shoulderToElbow = new Vector3(0, 1, 0);
                    double pitch = InverseKinematics.GetShoulderPitch(shoulderR2L, shoulderToElbow);

                    Assert.AreEqual(-90, pitch, angleTolerance);
                }
            }

            [TestClass]
            public class ShoulderRoll
            {
                [TestClass]
                public class ElbowPointingSide
                {
                    private static readonly Point3 elbow = new Point3(7, 10, 10);
                    private static readonly Vector3 shoulderToElbow = Vector3.FromPoints(shoulderL, elbow);
                    private static readonly double forwardFacingRatio = 0;

                    [TestMethod]
                    public void Inline()
                    {
                        Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll);
                    }

                    [TestMethod]
                    public void WristPointingDown()
                    {
                        Vector3 elbowToWrist = new Vector3(0, -1, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(-90, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingForward()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 0, -1);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingUp()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 1, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(90, roll, angleTolerance);
                    }
                }

                [TestClass]
                public class ElbowPointingDown
                {
                    private static readonly Point3 elbow = new Point3(8, 9, 10);
                    private static readonly Vector3 shoulderToElbow = Vector3.FromPoints(shoulderL, elbow);
                    private static readonly double forwardFacingRatio = 0;

                    [TestMethod]
                    public void Inline()
                    {
                        Vector3 elbowToWrist = new Vector3(0, -1, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll);
                    }

                    [TestMethod]
                    public void WristPointingForward()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 0, -1);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingInside()
                    {
                        Vector3 elbowToWrist = new Vector3(1, 0, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(-90, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingOutside()
                    {
                        Vector3 elbowToWrist = new Vector3(-1, 0, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(90, roll, angleTolerance);
                    }
                }

                [TestClass]
                public class ElbowPointingForward
                {
                    private static readonly Point3 elbow = new Point3(8, 10, 9);
                    private static readonly Vector3 shoulderToElbow = Vector3.FromPoints(shoulderL, elbow);
                    private static readonly double forwardFacingRatio = 1;

                    [TestMethod]
                    public void Inline()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 0, -1);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll);
                    }

                    [TestMethod]
                    public void WristPointingInside()
                    {
                        Vector3 elbowToWrist = new Vector3(1, 0, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(0, roll, angleTolerance);
                    }

                    [TestMethod]
                    public void WristPointingUp()
                    {
                        Vector3 elbowToWrist = new Vector3(0, 1, 0);
                        double roll = InverseKinematics.GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio);

                        Assert.AreEqual(90, roll, angleTolerance);
                    }
                }
            }

            [TestClass]
            public class ElbowPitch
            {
                private static readonly Vector3 shoulderToElbow = new Vector3(-1, 0, 0);

                [TestMethod]
                public void Inline()
                {
                    Vector3 elbowToWrist = new Vector3(-1, 0, 0);
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
                    Vector3 elbowToWrist = new Vector3(0, 1, 0);
                    double angle = InverseKinematics.GetElbowAngle(shoulderToElbow, elbowToWrist);

                    Assert.AreEqual(-90, angle, angleTolerance);
                }
            }
        }
    }
}

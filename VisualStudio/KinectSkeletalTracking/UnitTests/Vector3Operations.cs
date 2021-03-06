using Microsoft.VisualStudio.TestTools.UnitTesting;
using KinectSkeletalTracking;

namespace VectorTests
{
    [TestClass]
    public class Vector3Operations
    {
        [TestMethod]
        public void MultiplyValue()
        {
            Vector3 vector = new Vector3()
            {
                X = 1,
                Y = 2,
                Z = 3
            };

            Vector3 newVector = vector * 2;

            Assert.AreEqual(2, newVector.X);
            Assert.AreEqual(4, newVector.Y);
            Assert.AreEqual(6, newVector.Z);

            newVector = 2 * vector;

            Assert.AreEqual(2, newVector.X);
            Assert.AreEqual(4, newVector.Y);
            Assert.AreEqual(6, newVector.Z);
        }

        [TestMethod]
        public void SubtractVector()
        {
            Vector3 vector1 = new Vector3()
            {
                X = 3,
                Y = 3,
                Z = 3
            };

            Vector3 vector2 = new Vector3()
            {
                X = 2,
                Y = 2,
                Z = 2
            };

            Vector3 newVector = vector1 - vector2;

            Assert.AreEqual(1, newVector.X);
            Assert.AreEqual(1, newVector.Y);
            Assert.AreEqual(1, newVector.Z);
        }

        [TestMethod]
        public void Cross()
        {
            Vector3 vector1 = new Vector3()
            {
                X = 1,
                Y = 2,
                Z = 3
            };

            Vector3 vector2 = new Vector3()
            {
                X = 4,
                Y = 5,
                Z = 6
            };

            Vector3 newVector = vector1.Cross(vector2);

            Assert.AreEqual(-3, newVector.X);
            Assert.AreEqual(6, newVector.Y);
            Assert.AreEqual(-3, newVector.Z);
        }

        [TestMethod]
        public void Dot()
        {
            Vector3 vector1 = new Vector3()
            {
                X = 1,
                Y = 2,
                Z = 3
            };

            Vector3 vector2 = new Vector3()
            {
                X = 4,
                Y = 5,
                Z = 6
            };

            double dotProduct = vector1.Dot(vector2);

            Assert.AreEqual(32, dotProduct);
        }

        [TestMethod]
        public void AngleBetweenVectors()
        {
            Vector3 vector1 = new Vector3()
            {
                X = 1,
                Y = 1,
                Z = 0
            };

            // Use default values of 0
            Vector3 vector2 = new Vector3()
            {
                Y = 1
            };

            double angleDegrees = Vector3.GetAngleBetweenVectors(vector1, vector2);

            // Test fails when `angleDegrees` comes back as 45.0000000000000001,
            // as it is not 45 exact. Either cast `angleDegrees` to an int, or
            // allow a really small margin of error.
            Assert.AreEqual(45, angleDegrees, 0.0000000001);
        }
    }
}

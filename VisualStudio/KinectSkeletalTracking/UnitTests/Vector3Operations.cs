using Microsoft.VisualStudio.TestTools.UnitTesting;
using KinectSkeletalTracking;

namespace VectorTests
{
    [TestClass]
    public class Vector3Operations
    {
        [TestClass]
        public class Equality
        {
            private static readonly Vector3 comp = new Vector3(1, 1, 1);
            private static readonly Vector3 equal = new Vector3(1, 1, 1);
            private static readonly Vector3 notEqual = new Vector3(2, 2, 2);

            [TestMethod]
            public void EqualOperator()
            {
                Assert.IsTrue(comp == equal);
                Assert.IsTrue(comp != notEqual);
                Assert.IsFalse(comp == notEqual);
            }

            [TestMethod]
            public void EqualMethod()
            {
                Assert.IsTrue(comp.Equals(equal));
                Assert.IsFalse(comp.Equals(notEqual));
                Assert.IsFalse(comp.Equals(2)); // Random number, not a vector.
            }
        }

        [TestMethod]
        public void MultiplyValue()
        {
            Vector3 vector = new Vector3(1, 2, 3);
            Vector3 expectedVector = new Vector3(2, 4, 6);

            Vector3 newVector = vector * 2;

            Assert.AreEqual(expectedVector, newVector);

            newVector = 2 * vector;

            Assert.AreEqual(expectedVector, newVector);
        }

        [TestMethod]
        public void SubtractVector()
        {
            Vector3 vector1 = new Vector3(3,3,3);
            Vector3 vector2 = new Vector3(2, 2, 2);
            Vector3 expectedVector = new Vector3(1, 1, 1);

            Vector3 newVector = vector1 - vector2;

            Assert.AreEqual(expectedVector, newVector);
        }

        [TestMethod]
        public void Cross()
        {
            Vector3 vector1 = new Vector3(1, 2, 3);
            Vector3 vector2 = new Vector3(4, 5, 6);
            Vector3 expectedVector = new Vector3(-3, 6, -3);

            Vector3 newVector = vector1.Cross(vector2);

            Assert.AreEqual(expectedVector, newVector);
        }

        [TestMethod]
        public void Dot()
        {
            Vector3 vector1 = new Vector3(1, 2, 3);
            Vector3 vector2 = new Vector3(4, 5, 6);

            double dotProduct = vector1.Dot(vector2.X, vector2.Y, vector2.Z);

            Assert.AreEqual(32, dotProduct);

            dotProduct = vector1.Dot(vector2);

            Assert.AreEqual(32, dotProduct);
        }

        [TestMethod]
        public void AngleBetweenVectors()
        {
            Vector3 vector1 = new Vector3(1, 1, 0);
            Vector3 vector2 = new Vector3(0, 1, 0);

            double angleDegrees = Vector3.GetAngleBetweenVectors(vector1, vector2);

            // Test fails when `angleDegrees` comes back as 45.0000000000000001,
            // as it is not 45 exact. Either cast `angleDegrees` to an int, or
            // allow a really small margin of error.
            Assert.AreEqual(45, angleDegrees, 0.0000000001);
        }
    }
}

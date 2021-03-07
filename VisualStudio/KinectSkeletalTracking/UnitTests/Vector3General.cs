using Microsoft.VisualStudio.TestTools.UnitTesting;
using KinectSkeletalTracking;

namespace VectorTests
{
    [TestClass]
    public class Vector3General
    {
        [TestClass]
        public class Constructor
        {
            [TestMethod]
            public void ConstructorDefault()
            {
                Vector3 vector = new Vector3();

                // Default value
                Assert.AreEqual(0, vector.X);
                Assert.AreEqual(0, vector.Y);
                Assert.AreEqual(0, vector.Y);
            }

            [TestMethod]
            public void ConstructorValuesArguments()
            {
                Vector3 vector = new Vector3(1, 2, 3);

                Assert.AreEqual(1, vector.X);
                Assert.AreEqual(2, vector.Y);
                Assert.AreEqual(3, vector.Z);
            }

            [TestMethod]
            public void ConstructorValuesProperties()
            {
                Vector3 vector = new Vector3()
                {
                    X = 1,
                    Y = 2,
                    Z = 3
                };

                Assert.AreEqual(1, vector.X);
                Assert.AreEqual(2, vector.Y);
                Assert.AreEqual(3, vector.Z);
            }

            [TestMethod]
            public void ConstructorFromVector()
            {
                Vector3 otherVector = new Vector3(1, 2, 3);
                Vector3 vector = Vector3.FromVector(otherVector);

                Assert.AreEqual(otherVector.X, vector.X);
                Assert.AreEqual(otherVector.Y, vector.Y);
                Assert.AreEqual(otherVector.Z, vector.Z);
            }
        }

        [TestMethod]
        public void Magnitude()
        {
            Vector3 vector = new Vector3(0, 3, 4);

            Assert.AreEqual(5, vector.Magnitude);
        }
    }
}

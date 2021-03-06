using Microsoft.VisualStudio.TestTools.UnitTesting;
using KinectSkeletalTracking;

namespace VectorTests
{
    [TestClass]
    public class Vector3General
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
        public void ConstructorValues()
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
        public void Magnitude()
        {
            Vector3 vector = new Vector3()
            {
                X = 0,
                Y = 3,
                Z = 4
            };

            Assert.AreEqual(5, vector.Magnitude);
        }
    }
}

using KinectSkeletalTracking;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace UnitTests
{
    [TestClass]
    public class PlaneTests
    {
        [TestClass]
        public class Constructors
        {
            [TestMethod]
            public void Constructor()
            {
                Plane plane = new Plane(1, 2, 3, 4);

                Assert.AreEqual(1, plane.X);
                Assert.AreEqual(2, plane.Y);
                Assert.AreEqual(3, plane.Z);
                Assert.AreEqual(4, plane.D);
            }

            [TestMethod]
            public void FromPoints()
            {
                Plane plane = Plane.FromPoints(
                    new Point3(1, 0, 0),
                    new Point3(0, 1, 0),
                    new Point3(0, 0, 1)
                );

                Plane expectedPlane = new Plane(1, 1, 1, -1);

                Assert.AreEqual(expectedPlane, plane);
            }

            [TestMethod]
            public void FromVectors()
            {
                Plane plane = Plane.FromVectors(
                    new Vector3(1, 0, 0),
                    new Vector3(0, 1, 0)
                );

                Plane expectedPlane = new Plane(0, 0, 1, 0);

                Assert.AreEqual(expectedPlane, plane);
            }
        }

        [TestMethod]
        public void AsVector()
        {
            Plane plane = new Plane(1, 1, 1, -1);
            Vector3 vector = plane.AsVector();

            Vector3 expectedVector = new Vector3(1, 1, 1);

            Assert.AreEqual(expectedVector, vector);
        }

        [TestMethod]
        public void GetAngleBetweenPlanes()
        {
            Plane plane1 = new Plane(0, 0, 1, 0);
            Plane plane2 = new Plane(1, 0, 0, 0);

            double angle = Plane.GetAngleBetweenPlanes(plane1, plane2);

            Assert.AreEqual(90, angle);
        }
    }
}

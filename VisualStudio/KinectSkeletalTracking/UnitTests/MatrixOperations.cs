using MatrixDesign;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace MatrixTests
{
    [TestClass]
    public class MatrixOperations
    {
        [TestClass]
        public class Multiplication
        {
            [TestMethod]
            public void MultiplyByValue()
            {
                Matrix matrix = new Matrix(2, 3, new double[,]
                {
                    { 1, 2, 3 },
                    { 4, 5, 6 }
                });
                Matrix expectedMatrix = new Matrix(2, 3, new double[,]
                {
                    { 2, 4, 6 },
                    { 8, 10, 12 }
                });

                Matrix result = matrix * 2;

                Assert.AreEqual(expectedMatrix, result);
            }

            [TestMethod]
            public void MultiplyByIdentityMatrix()
            {
                Matrix matrix = new Matrix(3, 3, new double[,]
                {
                    { 1, 2, 3 },
                    { 4, 5, 6 },
                    { 7, 8, 9 }
                });

                Matrix identity = Matrix.Identity(3);

                Matrix result = matrix * identity;

                Assert.AreEqual(matrix, result);
            }

            [TestMethod]
            public void MultiplyByMatrix()
            {
                Matrix A = new Matrix(3, 3, new double[,]
                {
                    { 1, 2, 3 },
                    { 4, 5, 6 },
                    { 7, 8, 9 }
                });
                Matrix B = new Matrix(3, 1, new double[,]
                {
                    { 1 },
                    { 2 },
                    { 3 }
                });
                Matrix expectedMatrix = new Matrix(3, 1, new double[,]
                {
                    { 14 },
                    { 32 },
                    { 50 }
                });

                Matrix result = A * B;

                Assert.AreEqual(expectedMatrix, result);
            }
        }

        [TestClass]
        public class Addition
        {
            [TestMethod]
            public void AddValue()
            {
                Matrix matrix = new Matrix(2, 3, new double[,]
                {
                    { 1, 2, 3 },
                    { 4, 5, 6 }
                });
                Matrix expectedMatrix = new Matrix(2, 3, new double[,]
                {
                    { 3, 4, 5 },
                    { 6, 7, 8 }
                });

                Matrix result = matrix + 2;

                Assert.AreEqual(expectedMatrix, result);
            }

            [TestMethod]
            public void AddMatrix()
            {
                Matrix matrix = new Matrix(2, 3, new double[,]
                {
                    { 1, 2, 3 },
                    { 4, 5, 6 }
                });
                Matrix expectedMatrix = new Matrix(2, 3, new double[,]
                {
                    { 2, 4, 6 },
                    { 8, 10, 12 }
                });

                Matrix result = matrix + matrix; // Add itself

                Assert.AreEqual(expectedMatrix, result);
            }
        }

        [TestClass]
        public class Subtraction
        {
            [TestMethod]
            public void SubtractValue()
            {
                Matrix matrix = new Matrix(2, 3, new double[,]
                {
                    { 1, 2, 3 },
                    { 4, 5, 6 }
                });
                Matrix expectedMatrix = new Matrix(2, 3, new double[,]
                {
                    { -1, 0, 1 },
                    { 2, 3, 4 }
                });

                Matrix result = matrix - 2;

                Assert.AreEqual(expectedMatrix, result);
            }

            [TestMethod]
            public void SubtractMatrix()
            {
                Matrix matrix = new Matrix(2, 3, new double[,]
                {
                    { 1, 2, 3 },
                    { 4, 5, 6 }
                });
                Matrix expectedMatrix = new Matrix(2, 3, new double[,]
                {
                    { 0, 0, 0 },
                    { 0, 0, 0 }
                });

                Matrix result = matrix - matrix; // Subtract itself

                Assert.AreEqual(expectedMatrix, result);
            }
        }
    }
}

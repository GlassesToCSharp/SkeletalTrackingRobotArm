using MatrixDesign;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace MatrixTests
{
    [TestClass]
    public class Constructor
    {
        [TestMethod]
        public void RawConstruction()
        {
            Matrix matrix = new Matrix(1, 2, new double[,] {
                {3, 4 }
            });

            Assert.AreEqual(matrix.Rows, 1);
            Assert.AreEqual(matrix.Columns, 2);
            Assert.AreEqual(matrix[0, 0], 3);
            Assert.AreEqual(matrix[0, 1], 4);
        }

        [TestMethod]
        public void ZerosConstruction()
        {
            const int matrixSquareSize = 3;
            Matrix createdByIdentity = Matrix.Zeros(matrixSquareSize);
            Matrix createdManually = new Matrix(matrixSquareSize, matrixSquareSize, new double[,] {
                {0, 0, 0 },
                {0, 0, 0 },
                {0, 0, 0 }
            });

            Assert.AreEqual(createdManually, createdByIdentity);
        }

        [TestMethod]
        public void OnesConstruction()
        {
            const int matrixSquareSize = 3;
            Matrix createdByIdentity = Matrix.Ones(matrixSquareSize);
            Matrix createdManually = new Matrix(matrixSquareSize, matrixSquareSize, new double[,] {
                {1, 1, 1 },
                {1, 1, 1 },
                {1, 1, 1 }
            });

            Assert.AreEqual(createdManually, createdByIdentity);
        }

        [TestMethod]
        public void IdentityConstruction()
        {
            const int matrixSquareSize = 3;
            Matrix createdByIdentity = Matrix.Identity(matrixSquareSize);
            Matrix createdManually = new Matrix(matrixSquareSize, matrixSquareSize, new double[,] {
                {1, 0, 0 },
                {0, 1, 0 },
                {0, 0, 1 }
            });

            Assert.AreEqual(createdManually, createdByIdentity);
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MatrixDesign
{
    public class Matrix
    {
        // This iteration of the class needs to know the number of rows, number of columns, and the
        // the contents of the matrix (arranged as a 2D array of type double)
        int rows = 0;
        int columns = 0;
        double[,] matrixArray;


        #region Get/Sets for matrix


        // Publicly accessible read-only number of rows
        public int Rows => rows;


        // Publicly accessible read-only number of columns
        public int Columns => columns;


        // Publicly accessible individual matrix cells
        public double this[int row, int column]
        {
            get => matrixArray[row - 1, column - 1];
            set => matrixArray[row - 1, column - 1] = value;
        }


        // Publicly accessible read-only matrix row
        public double[] GetRow(int row)
        {
            if (row == 0)
            {
                throw new Exception("Row cannot be 0.");
            }

            row -= 1;
            double[] target = new double[columns];
            Buffer.BlockCopy(matrixArray, 8 * columns * row, target, 0, 8 * columns);
            return target;
        }


        // Publicly accessible read-only matrix column
        public double[] GetColumn(int column)
        {
            if (column == 0)
            {
                throw new Exception("Column cannot be 0.");
            }

            column -= 1;
            double[] target = new double[rows];
            int columnOffset = 8 * column;
            for (int i = 0; i < rows; i++)
            {
                Buffer.BlockCopy(matrixArray, columnOffset + i * columns * 8, target, i * 8, 8);
            }
            return target;
        }

        #endregion


        #region Initialisation variations


        public static Matrix Identity(int squareSize)
        {
            Matrix mat = new Matrix(squareSize, squareSize);

            mat.ApplyOperation((rowIndex, columnIndex) => rowIndex == columnIndex ? 1.0 : 0.0);

            return mat;
        }

        public static Matrix Zeros(int squareSize)
        {
            return new Matrix(squareSize, squareSize, initialValue: 0.0);
        }

        public static Matrix Ones(int squareSize)
        {
            return new Matrix(squareSize, squareSize, initialValue: 1.0);
        }


        // Initialiser for general matrices with a custom initial value set for all cells (default
        // option is a matrix of zeros).
        public Matrix(int rows, int columns, double initialValue = 0.0)
        {
            if (rows < 1 || columns < 1)
            {
                throw new Exception("Matrix cannot have 0 or negative-value rows/columns.");
            }

            this.rows = rows;
            this.columns = columns;
            matrixArray = new double[rows, columns];

            SetAllValuesTo(initialValue);
        }


        // Initialiser for specific rows/columns and specifc values for each cell.
        public Matrix(int rows, int columns, double[,] matrix)
        {
            if (rows < 1 || columns < 1 || matrix.Length < 1)
            {
                throw new Exception("Matrix cannot have 0 or negative-value rows/columns.");
            }
            else if (rows * columns < matrix.Length)
            {
                throw new Exception("Matrix rows and columns must match matrix given.");
            }

            this.rows = rows;
            this.columns = columns;
            matrixArray = matrix;
        }


        #endregion


        // Transpose the current matrix.
        public Matrix Transpose()
        {
            // switch the row/column value around
            Matrix temp = new Matrix(columns, rows);

            temp.ApplyOperation((matrixRow, matrixColumn) => this[matrixColumn, matrixRow]);

            return temp;
        }


        // Find the determinant of the current matrix. (Currently only for 2x2)
        public double Determinant()
        {
            double determinant;
            if (rows == columns && rows < 3)
            {
                determinant = (this[1, 1] * this[2, 2]) - (this[2, 1] * this[1, 2]);
            }
            else
            {
                throw new Exception("Cannot find determinant. Matrix needs to be square.");
            }

            return determinant;
        }


        // Multiply the current matrix by a number.
        public Matrix Multiply(double value)
        {
            Matrix resultantMatrix = new Matrix(rows, columns);

            resultantMatrix.ApplyOperation((matrixRow, matrixColumn) => this[matrixRow, matrixColumn] * value);

            return resultantMatrix;
        }


        // Multiply the current matrix by another matrix.
        public Matrix Multiply(Matrix matrix2)
        {
            Matrix matrix1 = this;
            Matrix resultantMatrix = new Matrix(matrix1.Rows, matrix2.Columns);

            if (matrix1.Columns != matrix2.Rows)
            {
                throw new Exception("Invalid matrices");
            }
            else
            {
                for (int matrix1Row = 1; matrix1Row <= matrix1.Rows; matrix1Row++)
                {
                    for (int matrix2Column = 1; matrix2Column <= matrix2.Columns; matrix2Column++)
                    {
                        var vector1 = matrix1.GetRow(matrix1Row);
                        var vector2 = matrix2.GetColumn(matrix2Column);

                        if (vector1.Length != vector2.Length)
                        {
                            throw new Exception("Vector lengths do not match.");
                        }

                        double result = 0;
                        for (int i = 0; i < vector1.Length; i++)
                        {
                            result += vector1[i] * vector2[i];
                        }

                        resultantMatrix[matrix1Row, matrix2Column] = result;
                    }
                }
            }

            return resultantMatrix;
        }


        // Add to the current matrix a number
        public Matrix Add(double value)
        {
            Matrix resultantMatrix = new Matrix(rows, columns);

            resultantMatrix.ApplyOperation((matrixRow, matrixColumn) => this[matrixRow, matrixColumn] + value);

            return resultantMatrix;
        }


        // Add to the current matrix another matrix
        public Matrix Add(Matrix matrix)
        {
            if (matrix.Rows != rows && matrix.Columns != columns)
            {
                throw new Exception("Invalid matrices");
            }

            Matrix resultantMatrix = new Matrix(rows, columns);

            resultantMatrix.ApplyOperation((matrixRow, matrixColumn) => this[matrixRow, matrixColumn] + matrix[matrixRow, matrixColumn]);

            return resultantMatrix;
        }


        // Subtract from the current matrix another matrix
        public Matrix Subtract(Matrix matrix)
        {
            if (matrix.Rows != rows && matrix.Columns != columns)
            {
                throw new Exception("Invalid matrices");
            }

            Matrix resultantMatrix = new Matrix(rows, columns);

            resultantMatrix.ApplyOperation((matrixRow, matrixColumn) => this[matrixRow, matrixColumn] - matrix[matrixRow, matrixColumn]);

            return resultantMatrix;
        }


        // Subtract from the current matrix a number
        public Matrix Subtract(double value)
        {
            return Add(value * -1);
        }


        // Privately set all the vallues of the current matrix to a specific value.
        private void SetAllValuesTo(double value)
        {
            ApplyOperation((_, __) => value);
        }


        private void ApplyOperation(Func<int, int, double> operation)
        {
            for (int rowIndex = 1; rowIndex <= rows; rowIndex++)
            {
                for (int columnIndex = 1; columnIndex <= columns; columnIndex++)
                {
                    this[rowIndex, columnIndex] = operation(rowIndex, columnIndex);
                }
            }
        }


        // Override the ToString() method, and print the matrix out in matrix notation.
        public override string ToString()
        {
            string outputString = "";
            for (int rowIndex = 1; rowIndex <= rows; rowIndex++)
            {
                for (int columnIndex = 1; columnIndex <= columns; columnIndex++)
                {
                    outputString += this[rowIndex, columnIndex].ToString("F3") + "\t";
                }

                outputString += "\r\n";
            }

            outputString += "\r\n";
            outputString += "\r\n";
            return outputString;
        }
    }
}

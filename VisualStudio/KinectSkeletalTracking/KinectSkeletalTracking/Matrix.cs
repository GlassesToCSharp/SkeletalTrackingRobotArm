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
        public int Rows
        {
            get { return rows; }
        }


        // Publicly accessible read-only number of columns
        public int Columns
        {
            get { return columns; }
        }


        // Publicly accessible individual matrix cells
        public double this[int row, int column] 
        {
            get { return matrixArray[row-1, column-1]; }
            set { matrixArray[row-1, column-1] = value; }
        }


        // Publicly accessible read-only matrix row
        public double[] GetRow(int row)
        {
            if(row == 0)
            {
                throw new Exception("Row cannot be 0.");
            }

            row = row - 1;
            double[] target = new double[this.Columns];
            Buffer.BlockCopy(this.matrixArray, 8 * this.Columns * row, target, 0, 8 * this.Columns);
            return target;
        }


        // Publicly accessible read-only matrix column
        public double[] GetColumn(int column)
        {
            if (column == 0)
            {
                throw new Exception("Column cannot be 0.");
            }

            column = column - 1;
            double[] target = new double[this.Rows];
            int columnOffset = 8 * column;
            for (int i = 0; i < this.Rows; i++)
            {
                Buffer.BlockCopy(this.matrixArray, columnOffset + i * this.Columns * 8, target, i * 8, 8);
            }
            return target;
        }

        #endregion


        #region Initialisation variations


        // Initialiser for square matrices (with option for contents to be the identity matrix). 
        // Default option is a square matrix of zeros.
        public Matrix(int squareRowsColumns, bool isIdentityMatrix = false)
        {
            NewMatrix(squareRowsColumns, squareRowsColumns);

            if (isIdentityMatrix)
            {
                for (int rowIndex = 1; rowIndex <= rows; rowIndex++)
                {
                    for (int columnIndex = 1; columnIndex <= columns; columnIndex++)
                    {
                        if (rowIndex == columnIndex)
                        {
                            this[rowIndex, columnIndex] = 1.0;
                        }
                        else
                        {
                            this[rowIndex, columnIndex] = 0.0;
                        }
                    }
                }
            }
            else
            {
                SetAllValuesTo(this, 0.0);
            }
        }


        // Initialiser for general matrices with a custom initial value set for all cells (default
        // option is a matrix of zeros).
        public Matrix(int rows, int columns, string initialValues)
        {
            NewMatrix(rows, columns);

            switch (initialValues.ToLower())
            {
                case "zero":
                case "zeros":
                default:
                    SetAllValuesTo(this, 0.0);
                    break;
                case "one":
                case "ones":
                    SetAllValuesTo(this, 1.0);
                    break;
            }
        }


        // Initialiser for general matrices with a custom initial value set for all cells (default
        // option is a matrix of zeros).
        public Matrix(int rows, int columns, double intialValues = 0.0)
        {
            NewMatrix(rows, columns);

            SetAllValuesTo(this, intialValues);
        }


        #endregion


        
        // General function to setup a new matrix for this instance.
        private void NewMatrix(int rows, int columns)
        {
            if (rows < 1 || columns < 1)
            {
                throw new Exception("Matrix cannot have 0 or negative-value rows/columns.");
            }

            this.rows = rows;
            this.columns = columns;
            matrixArray = new double[rows, columns];
        }

        
        // Set the content of the current matrix to a zero-matrix.
        public Matrix Zeros()
        {
            Matrix temp = new Matrix(rows, columns);

            SetAllValuesTo(temp, 0);

            return temp;
        }


        // Set the content of the current matrix to a random number between 0 and 1.
        public Matrix Random()
        {
            Matrix temp = new Matrix(rows, columns);
            System.Random random = new System.Random();

            for (int rowIndex = 1; rowIndex <= rows; rowIndex++)
            {
                for (int columnIndex = 1; columnIndex <= columns; columnIndex++)
                {
                    this[rowIndex, columnIndex] = random.NextDouble();
                }
            }

            return temp;
        }


        // Transpose the current matrix.
        public Matrix Transpose()
        {
            // switch the row/column value around
            Matrix temp = new Matrix(columns, rows);

            for(int rowIndex = 1; rowIndex <= columns; rowIndex++)
            {
                for(int columnIndex = 1; columnIndex <= rows; columnIndex++)
                {
                    temp[rowIndex, columnIndex] = this[columnIndex, rowIndex];
                }
            }

            return temp;
        }


        // Find the determinant of the current matrix. (Currently only for 2x2)
        public double Determinant()
        {
            double determinant = 0;
            if(rows == columns && rows < 3)
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
            Matrix resultantMatrix = this;

            for (int matrix1Row = 1; matrix1Row <= this.Rows; matrix1Row++)
            {
                for (int matrix2Column = 1; matrix2Column <= this.Columns; matrix2Column++)
                {
                    resultantMatrix[matrix1Row, matrix2Column] = this[matrix1Row, matrix2Column] * value;
                }
            }

            return resultantMatrix;
        }

        
        // Multiply the current matrix by another matrix.
        public Matrix Multiply(Matrix matrix2)
        {
            var matrix1 = this;
            Matrix resultantMatrix = new Matrix(matrix1.Rows, matrix2.Columns);

            if (matrix1.Columns != matrix2.Rows)
            {
                throw new Exception("Invalid matrices");
            }
            else
            {
                for(int matrix1Row = 1; matrix1Row <= matrix1.Rows; matrix1Row++)
                {
                    for(int matrix2Column = 1; matrix2Column <= matrix2.Columns; matrix2Column++)
                    {
                        var vector1 = matrix1.GetRow(matrix1Row);
                        var vector2 = matrix2.GetColumn(matrix2Column);

                        if(vector1.Length != vector2.Length)
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
            Matrix resultantMatrix = new Matrix(this.Rows, this.Columns);

            for (int matrixRow = 1; matrixRow <= this.Rows; matrixRow++)
            {
                for (int matrixColumn = 1; matrixColumn <= this.Columns; matrixColumn++)
                {
                    resultantMatrix[matrixRow, matrixColumn] = this[matrixRow, matrixColumn] + value;
                }
            }

            return resultantMatrix;
        }


        // Add to the current matrix another matrix
        public Matrix Add(Matrix matrix)
        {
            if(matrix.Rows != this.Rows && matrix.Columns != this.Columns)
            {
                throw new Exception("Invalid matrices");
            }

            Matrix resultantMatrix = new Matrix(this.Rows, this.Columns);

            for (int matrixRow = 1; matrixRow <= this.Rows; matrixRow++)
            {
                for (int matrixColumn = 1; matrixColumn <= this.Columns; matrixColumn++)
                {
                    resultantMatrix[matrixRow, matrixColumn] = this[matrixRow, matrixColumn] + matrix[matrixRow, matrixColumn];
                }
            }

            return resultantMatrix;
        }


        // Subtract from the current matrix another matrix
        public Matrix Subtract(Matrix matrix)
        {
            if(matrix.Rows != this.Rows && matrix.Columns != this.Columns)
            {
                throw new Exception("Invalid matrices");
            }

            Matrix resultantMatrix = new Matrix(this.Rows, this.Columns);

            for (int matrixRow = 1; matrixRow <= this.Rows; matrixRow++)
            {
                for (int matrixColumn = 1; matrixColumn <= this.Columns; matrixColumn++)
                {
                    resultantMatrix[matrixRow, matrixColumn] = this[matrixRow, matrixColumn] - matrix[matrixRow, matrixColumn];
                }
            }

            return resultantMatrix;
        }


        // Subtract from the current matrix a number
        public Matrix Subtract(double value)
        {
            return this.Add(value * -1);
        }


        // Privately set all the vallues of the current matrix to a specific value.
        private void SetAllValuesTo(Matrix matrix, double value)
        {
            int rows = matrix.Rows;
            int columns = matrix.Columns;

            for(int rowIndex = 1; rowIndex <= rows; rowIndex++)
            {
                for(int columnIndex = 1; columnIndex <= columns; columnIndex++)
                {
                    matrix[rowIndex, columnIndex] = value;
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

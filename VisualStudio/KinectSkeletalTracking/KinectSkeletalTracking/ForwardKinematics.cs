using MatrixDesign;
using System;
using System.Linq;

namespace KinectSkeletalTracking
{
    public class ForwardKinematics : IEquatable<ForwardKinematics>
    { 
        public Point3 Shoulder { get; private set; }
        public Point3 Elbow { get; private set; }
        public Point3 Wrist { get; private set; }

        /// <summary>
        /// Creates an instance of <c>ForwardKinematics</c>.
        /// </summary>
        /// <param name="s">The shoulder point.</param>
        /// <param name="e">The elbow point.</param>
        /// <param name="w">The wrist point.</param>
        public ForwardKinematics(Point3 s, Point3 e, Point3 w)
        {
            Shoulder = s;
            Elbow = e;
            Wrist = w;
        }

        #region Operations

        public static bool operator ==(ForwardKinematics thisFK, ForwardKinematics otherFK)
        {
            return thisFK.Shoulder == otherFK.Wrist &&
                thisFK.Elbow == otherFK.Elbow &&
                thisFK.Wrist == otherFK.Wrist;
        }

        public static bool operator !=(ForwardKinematics thisFK, ForwardKinematics otherFK)
        {
            return !(thisFK == otherFK);
        }

        public override bool Equals(object obj)
        {
            return obj != null && obj is ForwardKinematics && Equals(obj as ForwardKinematics);
        }

        public bool Equals(ForwardKinematics other)
        {
            return this == other;
        }

        public override int GetHashCode()
        {
            unchecked // Overflow is fine, just wrap
            {
                int hash = 17;
                hash = hash * 23 + Shoulder.GetHashCode();
                hash = hash * 23 + Elbow.GetHashCode();
                hash = hash * 23 + Wrist.GetHashCode();
                return hash;
            }
        }

        #endregion

        /// <summary>
        /// Retrieves the forward kinematics given the shoulder position and the inverse kinematics.
        /// </summary>
        /// <param name="shoulderPosition">The position of the "base" shoulder.</param>
        /// <param name="ik">The inverse kinematics containing the angles of the joints.</param>
        /// <returns>The forward kinamtic result given the inverse kinematic angles.</returns>
        public static ForwardKinematics GetForwardKinematics(Point3 shoulderPosition, InverseKinematics ik)
        {
            // The angles are required to be in radians. Just in case they are not, convert to radians.
            InverseKinematics inRadians = ik.ToRadians();

            return GetForwardKinematics(shoulderPosition, inRadians.ShoulderYaw, inRadians.ShoulderPitch, inRadians.ShoulderRoll, inRadians.ElbowPitch, inRadians.UpperArmLength, inRadians.LowerArmLength);
        }

        /// <summary>
        /// Retrieves the forward kinematics given the shoulder position and the given angles.
        /// </summary>
        /// <param name="shoulderPosition">The position of the "base" shoulder.</param>
        /// <param name="shoulderYawRadians">The angle of the shoulder yaw in radians.</param>
        /// <param name="shoulderPitchRadians">The angle of the shoulder pitch in radians.</param>
        /// <param name="shoulderRollRadians">The angle of the shoulder roll in radians.</param>
        /// <param name="elbowPitchRadians">The angle of the elbow pitch in radians</param>
        /// <param name="upperArmLength">The length of the upper arm.</param>
        /// <param name="lowerArmLength">The length of the lower arm.</param>
        /// <returns></returns>
        public static ForwardKinematics GetForwardKinematics(Point3 shoulderPosition, double shoulderYawRadians, double shoulderPitchRadians, double shoulderRollRadians, double elbowPitchRadians, double upperArmLength, double lowerArmLength)
        {
            Matrix elbowMatrix = GetElbowTranslationMatrix(shoulderPosition, upperArmLength, shoulderYawRadians, shoulderPitchRadians);
            Point3 elbow = GetElbowPoint(shoulderPosition, upperArmLength, shoulderYawRadians, shoulderPitchRadians, elbowMatrix: elbowMatrix);
            Point3 wrist = GetWristPoint(elbowMatrix, shoulderRollRadians - shoulderYawRadians, elbowPitchRadians, lowerArmLength);

            return new ForwardKinematics(shoulderPosition, elbow, wrist);
        }

        /// <summary>
        /// Gets the point of the elbow from the given shoulder position, and the given angles and arm length.
        /// </summary>
        /// <param name="shoulderPosition">The point to the shoulder associated to the arm.</param>
        /// <param name="upperArmLength">The length of the upper arm.</param>
        /// <param name="shoulderYawRadians">The angle of the shoulder yaw in radians.</param>
        /// <param name="shoulderPitchRadians">The angle of the shoulder pitch in radians.</param>
        /// <param name="elbowMatrix">[Optional] The forward kinematic matrix to get to the elbow position.</param>
        /// <returns>The point of the elbow from the shoulder.</returns>
        public static Point3 GetElbowPoint(Point3 shoulderPosition, double upperArmLength, double shoulderYawRadians, double shoulderPitchRadians, Matrix elbowMatrix = null)
        {
            Matrix matrix = elbowMatrix ?? GetElbowTranslationMatrix(shoulderPosition, upperArmLength, shoulderYawRadians, shoulderPitchRadians);
            return GetPositionPointFromMatrix(matrix);
        }


        /// <summary>
        /// Gets the point of the elbow from the given elbow translation matrix, and the given angles and arm length.
        /// </summary>
        /// <param name="elbowTranslationMatrix">The translation matrix from base to elbow.</param>
        /// <param name="shoulderRollRadians">The angle of the shoulder roll in radians.</param>
        /// <param name="elbowPitchRadians">The angle of the elbow pitch in radians</param>
        /// <param name="lowerArmLength">The length of the lower arm.</param>
        /// <returns>The point of the wrist from the origin of the elbow translation matrix.</returns>
        public static Point3 GetWristPoint(Matrix elbowTranslationMatrix, double shoulderRollRadians, double elbowPitchRadians, double lowerArmLength)
        {
            Matrix t03 = elbowTranslationMatrix;

            // Base - elbow point.
            // T34 - Translation from base to shoulder roll (revolution about X axis)
            // T45 - Translation from shoulder roll to elbow pitch (revolution about Y axis)
            // T5e - Translation from elbow pitch to end effector (translation through X axis)
            // T3e - Translation from base to end effector
            //  L2 - Length from elbow to end effector

            double l2 = lowerArmLength;
            double theta3 = shoulderRollRadians;
            double theta4 = elbowPitchRadians;

            Matrix t34 = new Matrix(4, 4, new double[,] {
                    { 1, 0, 0, 0 },
                    { 0, Math.Cos(theta3), -Math.Sin(theta3), 0 },
                    { 0, Math.Sin(theta3), Math.Cos(theta3), 0 },
                    { 0, 0, 0, 1 },
                });
            Matrix t45 = new Matrix(4, 4, new double[,] {
                    { Math.Cos(theta4), 0, -Math.Sin(theta4), 0 },
                    { 0, 1, 0, 0 },
                    { Math.Sin(theta4), 0, Math.Cos(theta4), 0 },
                    { 0, 0, 0, 1 },
                });
            Matrix t5e = new Matrix(4, 4, new double[,] {
                    { 1, 0, 0, l2 },
                    { 0, 1, 0, 0 },
                    { 0, 0, 1, 0 },
                    { 0, 0, 0, 1 },
                });

            Matrix t3e = t34 * t45 * t5e;
            Matrix t0e = t03 * t3e;
            Point3 wristPosition = GetPositionPointFromMatrix(t0e);

            return wristPosition;
        }


        /// <summary>
        /// Gets the DH Matrix required to translate from the base to the elbow joint.
        /// </summary>
        /// <param name="shoulderPosition">The point to the shoulder associated to the arm.</param>
        /// <param name="upperArmLength">The length of the upper arm.</param>
        /// <param name="shoulderYawRadians">The angle of the shoulder yaw in radians.</param>
        /// <param name="shoulderPitchRadians">The angle of the shoulder pitch in radians.</param>
        /// <returns>The translation matrix from base to elbow.</returns>
        public static Matrix GetElbowTranslationMatrix(Point3 shoulderPosition, double upperArmLength, double shoulderYawRadians, double shoulderPitchRadians)
        {
            // Base - right shoulder point.
            // T01 - Translation from base to shoulder yaw (revolution about X axis)
            // T12 - Translation from shoulder yaw to shoulder pitch (revolution about Z axis)
            // T23 - Translation from shoulder pitch to elbow joint (translation through X axis)
            // T03 - Translation from base to elbow joint
            //  L1 - Length from shoulder to elbow

            double l1 = upperArmLength;
            double theta1 = shoulderYawRadians;
            double theta2 = shoulderPitchRadians;

            Matrix rightShoulderOrigin = new Matrix(4, 4, new double[,] {
                    { 1, 0, 0, shoulderPosition.X },
                    { 0, 1, 0, shoulderPosition.Y },
                    { 0, 0, 1, shoulderPosition.Z },
                    { 0, 0, 0, 1 }
                });

            Matrix t01 = rightShoulderOrigin * new Matrix(4, 4, new double[,] {
                    { 1, 0, 0, 0 },
                    { 0, Math.Cos(theta1), -Math.Sin(theta1), 0 },
                    { 0, Math.Sin(theta1), Math.Cos(theta1), 0 },
                    { 0, 0, 0, 1 },
                });
            Matrix t12 = new Matrix(4, 4, new double[,] {
                    { Math.Cos(theta2), 0, -Math.Sin(theta2), 0 },
                    { 0, 1, 0, 0 },
                    { Math.Sin(theta2), 0, Math.Cos(theta2), 0 },
                    { 0, 0, 0, 1 },
                });
            Matrix t23 = new Matrix(4, 4, new double[,] {
                    { 1, 0, 0, l1 },
                    { 0, 1, 0, 0 },
                    { 0, 0, 1, 0 },
                    { 0, 0, 0, 1 },
                });

            Matrix t03 = t01 * t12 * t23;
            return t03;
        }


        /// <summary>
        /// Retrieves the position from the translation matrix.
        /// </summary>
        /// <param name="matrix">The translation matrix containing the position. Must be at least 4x4.</param>
        /// <returns>The position point.</returns>
        private static Point3 GetPositionPointFromMatrix(Matrix matrix)
        {
            if (matrix.Rows < 3 || matrix.Columns < 3)
            {
                throw new Exception("Not enough columns or rows.");
            }

            return new Point3(
                matrix.GetRow(0).Last(),
                matrix.GetRow(1).Last(),
                matrix.GetRow(2).Last()
            );
        }
    }
}

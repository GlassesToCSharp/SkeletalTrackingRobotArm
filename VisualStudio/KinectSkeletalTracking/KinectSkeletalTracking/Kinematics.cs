using MatrixDesign;
using Microsoft.Kinect;
using System;
using System.Linq;

namespace KinectSkeletalTracking
{
    public static class Kinematics
    {
        /// <summary>
        /// Calculates the shoulder pitch.
        /// </summary>
        /// <param name="crossShoulder">The vector from left to right shoulder (or right to left).</param>
        /// <param name="shoulderToElbow">The vector from the shoulder (left or right) to the joining elbow.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the shoulder pitch.</returns>
        public static double GetShoulderPitch(Vector3 crossShoulder, Vector3 shoulderToElbow, bool inRadians = false)
        {
            return Vector3.GetAngleBetweenVectors(crossShoulder, shoulderToElbow, inRadians);
        }


        /// <summary>
        /// Calculates the shoulder yaw.
        /// </summary>
        /// <param name="spine">The vector of the spine.</param>
        /// <param name="crossShoulder">The vector from left to right shoulder (or right to left).</param>
        /// <param name="shoulderToElbow">The vector from the shoulder (left or right) to the joining elbow.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the shoulder yaw.</returns>
        public static double GetShoulderYaw(Vector3 spine, Vector3 crossShoulder, Vector3 shoulderToElbow, bool inRadians = false)
        {
            // 1. Get perpendicular vector of the cross-shoulder and shoulderElbow vectors.
            // 2. Get perpendicular vector of body plane.
            // 3. Get angle between perpendicular vectors.
            // 4. 0 degrees => perpendicular vectors are perpendicular to each
            //    other. Negative angle => elbow is pointing to feet. Positive
            //    angle => to the sky.

            // Get perpendicular vector by doing cross product of crossShoulder and spine
            Vector3 bodyPerpendicular = crossShoulder.Cross(spine) * -1;
            Vector3 armPerpendicular = crossShoulder.Cross(shoulderToElbow);

            double yaw = Vector3.GetAngleBetweenVectors(bodyPerpendicular, armPerpendicular, inRadians);

            if (Double.IsNaN(yaw))
            {
                // TODO: I'm not 100% sure on how to handle a NaN event. This
                // would only happen if the upper arm and shoulders are lined
                // up perfectly, resulting in trying to get a perpendicular
                // vector of two vectors that line up exactly, which has
                // infinite possibilities.
                // For now, return 0, as the angle wouldn't matter if the
                // vectors line up.
                return 0.0;
            }

            // We now need to determine if the elbow is above or below the 
            // shoulder. Get the angle between the armPerpenicular and spine
            // vectors. If greater than 90 degrees, elbow is below shoulder. 
            // Therefore, make negative.
            double signDeterminant = Vector3.GetAngleBetweenVectors(armPerpendicular, spine, inRadians);
            if (signDeterminant > (inRadians ? Math.PI / 2 : 90))
            {
                yaw = -yaw;
            }

            return yaw;
        }


        /// <summary>
        /// Calculates the shoulder roll.
        /// </summary>
        /// <param name="crossShoulder">The vector from left to right shoulder (or right to left).</param>
        /// <param name="shoulderToElbow">The vector from the shoulder (left or right) to the joining elbow.</param>
        /// <param name="elbowToWrist">The vector from the elbow (attached to joining shoulder) to the joining wrist.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the shoulder roll.</returns>
        public static double GetShoulderRoll(Vector3 crossShoulder, Vector3 shoulderToElbow, Vector3 elbowToWrist, bool inRadians = false)
        {
            // 1. Get shoulder-shoulder-elbow plane (SSE)
            // 2. Get shoulder-elbow-wrist plane (SEW)
            // 3. Get SSE orthogonal vector
            // 4. Get SEW orthogonal vector
            // 5. Get vector angles

            // Get planes
            Plane armPlane = Plane.FromVectors(new Vector3[]
            {
                shoulderToElbow * -1, // Why am I multiplying by -1??
                elbowToWrist
            });
            Plane bodyPlane = Plane.FromVectors(new Vector3[]
            {
                crossShoulder,
                shoulderToElbow
            });

            // Get orthogonal vectors
            Vector3 bodyOrthogonal = Vector3.FromVector(bodyPlane);
            Vector3 armOrthogonal = Vector3.FromVector(armPlane);

            // Calculate angle between orthogonal vector of Body and orthogonal vector of Arm
            double roll = Vector3.GetAngleBetweenVectors(bodyOrthogonal, armOrthogonal, inRadians);

            if (Double.IsNaN(roll))
            {
                // TODO: I'm not 100% sure on how to handle a NaN event. This
                // would only happen if the upper arm and shoulders are lined
                // up perfectly, resulting in trying to get a perpendicular
                // vector of two vectors that line up exactly, which has
                // infinite possibilities.
                // For now, return 0, as the angle wouldn't matter if the
                // vectors line up.
                return 0.0;
            }

            return roll;
        }


        /// <summary>
        /// Calculates the elbow angle.
        /// </summary>
        /// <param name="shoulderToElbow">The vector from the shoulder (left or right) to the joining elbow.</param>
        /// <param name="elbowToWrist">The vector from the elbow (attached to joining shoulder) to the joining wrist.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the elbow pitch.</returns>
        public static double GetElbowAngle(Vector3 shoulderToElbow, Vector3 elbowToWrist, bool inRadians = false)
        {
            return Vector3.GetAngleBetweenVectors(shoulderToElbow, elbowToWrist, inRadians);
        }


        /// <summary>
        /// Gets the vector of the elbow from the given shoulder position, and the given angles and arm length.
        /// </summary>
        /// <param name="shoulderPosition">The point to the shoulder associated to the arm.</param>
        /// <param name="upperArmLength">The length of the upper arm.</param>
        /// <param name="shoulderYawRadians">The angle of the shoulder yaw in radians.</param>
        /// <param name="shoulderPitchRadians">The angle of the shoulder pitch in radians.</param>
        /// <returns>The vector of the elbow from the shoulder.</returns>
        public static Point3 GetElbowPoint(Point3 shoulderPosition, double upperArmLength, double shoulderYawRadians, double shoulderPitchRadians)
        {
            Matrix t03 = GetElbowTranslationMatrix(shoulderPosition, upperArmLength, shoulderYawRadians, shoulderPitchRadians);
            return GetPositionPointFromMatrix(t03);
        }


        /// <summary>
        /// Gets the vector of the elbow from the given elbow translation matrix, and the given angles and arm length.
        /// </summary>
        /// <param name="elbowTranslationMatrix">The translation matrix from base to elbow.</param>
        /// <param name="shoulderRollRadians">The angle of the shoulder roll in radians.</param>
        /// <param name="elbowPitchRadians">The angle of the elbow pitch in radians</param>
        /// <param name="lowerArmLength">The length of the lower arm.</param>
        /// <returns>The vector of the wrist from the origin of the elbow translation matrix.</returns>
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
        /// Retrieves the position vector from the translation matrix.
        /// </summary>
        /// <param name="matrix">The translation matrix containing the position vector. Must be at least 4x4.</param>
        /// <returns>The position point.</returns>
        private static Point3 GetPositionPointFromMatrix(Matrix matrix)
        {
            if (matrix.Rows < 3 || matrix.Columns < 3)
            {
                throw new Exception("Not enough columns or rows.");
            }

            return new Point3
            {
                X = matrix.GetRow(0).Last(),
                Y = matrix.GetRow(1).Last(),
                Z = matrix.GetRow(2).Last()
            };
        }
    }
}

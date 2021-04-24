using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    public class InverseKinematics : IEquatable<InverseKinematics>
    {
        const double deg2rad = Math.PI / 180;
        const double rad2deg = 180 / Math.PI;

        public bool IsInRadians { get; private set; }
        public double ShoulderYaw { get; private set; }
        public double ShoulderPitch { get; private set; }
        public double ShoulderRoll { get; private set; }
        public double ElbowPitch { get; private set; }
        public double UpperArmLength { get; private set; }
        public double LowerArmLength { get; private set; }

        /// <summary>
        /// Creates an instance of <c>InverseKinematics</c>.
        /// </summary>
        /// <param name="sy">Shoulder yaw</param>
        /// <param name="sp">Shoulder pitch</param>
        /// <param name="sr">Shoulder roll</param>
        /// <param name="ep">Elbow pitch</param>
        /// <param name="ual">Upper arm length</param>
        /// <param name="lal">Lower arm length</param>
        /// <param name="isInRadians">Whether the angles provided are in radians.</param>
        public InverseKinematics(double sy, double sp, double sr, double ep, double ual, double lal, bool isInRadians = true)
        {
            IsInRadians = isInRadians;
            ShoulderPitch = sp;
            ShoulderRoll = sr;
            ShoulderYaw = sy;
            ElbowPitch = ep;
            UpperArmLength = ual;
            LowerArmLength = lal;
        }

        #region Operations

        public static bool operator ==(InverseKinematics thisIK, InverseKinematics otherIK)
        {
            return thisIK.IsInRadians == otherIK.IsInRadians &&
                thisIK.ShoulderYaw == otherIK.ShoulderYaw &&
                thisIK.ShoulderPitch == otherIK.ShoulderPitch &&
                thisIK.ShoulderRoll == otherIK.ShoulderRoll &&
                thisIK.ElbowPitch == otherIK.ElbowPitch;
        }

        public static bool operator !=(InverseKinematics thisIK, InverseKinematics otherIK)
        {
            return !(thisIK == otherIK);
        }

        public override bool Equals(object obj)
        {
            return obj != null && obj is InverseKinematics && Equals(obj as InverseKinematics);
        }

        public bool Equals(InverseKinematics other)
        {
            return this == other;
        }

        public override int GetHashCode()
        {
            unchecked // Overflow is fine, just wrap
            {
                int hash = 17;
                hash = hash * 23 + ShoulderYaw.GetHashCode();
                hash = hash * 23 + ShoulderPitch.GetHashCode();
                hash = hash * 23 + ShoulderRoll.GetHashCode();
                hash = hash * 23 + ElbowPitch.GetHashCode();
                hash = hash * 23 + IsInRadians.GetHashCode();
                return hash;
            }
        }

        #endregion

        /// <summary>
        /// Converts angles to radians, if they are not already in radians.
        /// </summary>
        /// <returns>Returns a new instance with the radian-equivalent angles.</returns>
        public InverseKinematics ToRadians()
        {
            if (IsInRadians)
            {
                return this;
            }

            return new InverseKinematics(ShoulderYaw * deg2rad, ShoulderPitch * deg2rad, ShoulderRoll * deg2rad, ElbowPitch * deg2rad, UpperArmLength, LowerArmLength, isInRadians: true);
        }

        /// <summary>
        /// Converts angles to degrees, if they are not already in degrees.
        /// </summary>
        /// <returns>Returns a new instance with the degree-equivalent angles.</returns>
        public InverseKinematics ToDegrees()
        {
            if (!IsInRadians)
            {
                return this;
            }

            return new InverseKinematics(ShoulderYaw * rad2deg, ShoulderPitch * rad2deg, ShoulderRoll * rad2deg, ElbowPitch * rad2deg, UpperArmLength, LowerArmLength, isInRadians: false);
        }

        /// <summary>
        /// Retreives the inverse kinematic angles given the points of the joints.
        /// </summary>
        /// <param name="neck">Neck point</param>
        /// <param name="spine">Spine point</param>
        /// <param name="shoulderL">Left shoulder point</param>
        /// <param name="shoulderR">Right shoulder point</param>
        /// <param name="elbow">Elbow point</param>
        /// <param name="wrist">Wrist point</param>
        /// <param name="inRadians">Whether the <c>InverseKinematics</c> should be in radians.</param>
        /// <returns>An instance of <c>InverseKinematics</c> with the determined angles.</returns>
        public static InverseKinematics GetInverseKinematicsRight(Point3 neck, Point3 spine, Point3 shoulderL, Point3 shoulderR, Point3 elbow, Point3 wrist, bool inRadians = false)
        {
            Plane body = Plane.FromPoints(shoulderR, shoulderL, spine);
            Vector3 neckToSpine = Vector3.FromPoints(neck, spine);
            Vector3 shoulderL2R = Vector3.FromPoints(shoulderL, shoulderR);
            Vector3 shoulderToElbow = Vector3.FromPoints(shoulderR, elbow);
            Vector3 elbowToWrist = Vector3.FromPoints(elbow, wrist);
            double elbowDistanceFromBody = elbow.DistanceFromPlane(body);

            double shoulderYaw = GetShoulderYaw(neckToSpine, shoulderL2R, shoulderToElbow, inRadians: inRadians);
            double shoulderPitch = GetShoulderPitch(shoulderL2R, shoulderToElbow, inRadians: inRadians);
            double forwardFacingRatio = (elbowDistanceFromBody / shoulderToElbow.Magnitude) * -1; // Multiply by -1 to account for negative perpedicularity.
            double shoulderRoll = GetShoulderRollRight(neckToSpine, shoulderL, shoulderR, elbow, wrist, inRadians: inRadians);
            double elbowPitch = GetElbowAngle(shoulderToElbow, elbowToWrist, inRadians: inRadians);

            return new InverseKinematics(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch, shoulderToElbow.Magnitude, elbowToWrist.Magnitude, isInRadians: inRadians);
        }

        public static InverseKinematics GetInverseKinematicsLeft(Point3 neck, Point3 spine, Point3 shoulderL, Point3 shoulderR, Point3 elbow, Point3 wrist, bool inRadians = false)
        {
            Plane body = Plane.FromPoints(shoulderR, shoulderL, spine);
            Vector3 neckToSpine = Vector3.FromPoints(neck, spine);
            Vector3 shoulderR2L = Vector3.FromPoints(shoulderR, shoulderL);
            Vector3 shoulderToElbow = Vector3.FromPoints(shoulderL, elbow);
            Vector3 elbowToWrist = Vector3.FromPoints(elbow, wrist);
            double elbowDistanceFromBody = body.AsVector().Dot(elbow.X, elbow.Y, elbow.Z);
            elbowDistanceFromBody += body.D;
            elbowDistanceFromBody /= body.AsVector().Magnitude;

            double shoulderYaw = GetShoulderYaw(neckToSpine, shoulderR2L, shoulderToElbow, inRadians: inRadians);
            double shoulderPitch = -GetShoulderPitch(shoulderR2L, shoulderToElbow, inRadians: inRadians);
            double forwardFacingRatio = (elbowDistanceFromBody / shoulderToElbow.Magnitude) * -1; // Multiply by -1 to account for negative perpedicularity.
            double shoulderRoll = GetShoulderRollLeft(neckToSpine, shoulderR2L, shoulderToElbow, elbowToWrist, forwardFacingRatio, inRadians: inRadians);
            double elbowPitch = -GetElbowAngle(shoulderToElbow, elbowToWrist, inRadians: inRadians);

            return new InverseKinematics(shoulderYaw, shoulderPitch, shoulderRoll, elbowPitch, shoulderToElbow.Magnitude, elbowToWrist.Magnitude, isInRadians: inRadians);
        }

        /// <summary>
        /// Calculates the shoulder pitch.
        /// </summary>
        /// <param name="shoulderL2R">The vector from left to right shoulder (or right to left).</param>
        /// <param name="shoulderToElbow">The vector from the shoulder (left or right) to the joining elbow.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the shoulder pitch.</returns>
        public static double GetShoulderPitch(Vector3 shoulderL2R, Vector3 shoulderToElbow, bool inRadians = false)
        {
            // Angle must be in the range of 0 (neutral) to -90 (close to body).
            return -Vector3.GetAngleBetweenVectors(shoulderL2R, shoulderToElbow, inRadians);
        }


        /// <summary>
        /// Calculates the shoulder yaw.
        /// </summary>
        /// <param name="neckToSpine">The vector from the neck point the spine point.</param>
        /// <param name="shoulderL2R">The vector from left to right shoulder (or right to left).</param>
        /// <param name="shoulderToElbow">The vector from the shoulder (left or right) to the joining elbow.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the shoulder yaw.</returns>
        public static double GetShoulderYaw(Vector3 neckToSpine, Vector3 shoulderL2R, Vector3 shoulderToElbow, bool inRadians = false)
        {
            // 1. Get perpendicular vector of the cross-shoulder and shoulderElbow vectors.
            // 2. Get perpendicular vector of body plane.
            // 3. Get angle between perpendicular vectors.
            // 4. 0 degrees => perpendicular vectors are perpendicular to each
            //    other. Negative angle => elbow is pointing to feet. Positive
            //    angle => to the sky.

            // Get perpendicular vector by doing cross product of crossShoulder and spine
            Vector3 bodyPerpendicular = shoulderL2R.Cross(neckToSpine);
            Vector3 armPerpendicular = shoulderL2R.Cross(shoulderToElbow);

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

            // To get the angle range -90 to 90, we need to subtract 90 (or rad equivalent).
            return yaw - (inRadians ? Math.PI / 2 : 90);
        }


        /// <summary>
        /// Calculates the shoulder roll for the right arm.
        /// </summary>
        /// <param name="neckToSpine">The vector from the neck point to the spine point.</param>
        /// <param name="shoulderL2R">The vector from left to right shoulder.</param>
        /// <param name="shoulderToElbow">The vector from the right shoulder to the right elbow.</param>
        /// <param name="elbowToWrist">The vector from the right elbow to the right wrist.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the shoulder roll.</returns>
        public static double GetShoulderRollRight(Vector3 neckToSpine, Point3 shoulderL, Point3 shoulderR, Point3 elbow, Point3 wrist, bool inRadians = false)
        {
            Vector3 shoulderL2R = Vector3.FromPoints(shoulderL, shoulderR);

            // 1. Get Body plane (neck-spine and shoulder-shoulder).
            // 2. Get upper arm plane (shoulder-shoulder and shoulder-elbow).
            // 3. Get Arm plane (shoulder-elbow and elbow-wrist).
            // 4. Get angle of planes.
            // 5. Adapt angle to be in the range of 90 deg (pointing up) to -90
            // (pointing down).

            Plane upperArmPlane = Plane.FromPoints(shoulderL, shoulderR, elbow);
            Plane armPlane = Plane.FromPoints(shoulderR, elbow, wrist);

            if (armPlane.AsVector().IsEmpty)
            {
                return 0;
            }

            double angle;
            if (upperArmPlane.AsVector().IsEmpty)
            {
                // When the elbow is point perpendicular to the body to the
                // side, the upper arm plane will be empty (as the shoulders
                // and upper arm vectors will line up). Use the body plane's
                // normal vector to create a new virtual should to create an
                // upper arm plane.
                Plane bodyPlane = Plane.FromVectors(neckToSpine, shoulderL2R);
                Vector3 bodyPlaneNormal = bodyPlane.AsVector();
                Point3 shoulderOffset = new Point3(shoulderR.X + bodyPlaneNormal.X, shoulderR.Y + bodyPlaneNormal.Y, shoulderR.Z + bodyPlaneNormal.Z);
                upperArmPlane = Plane.FromPoints(shoulderR, shoulderL, shoulderOffset);
                angle = Plane.GetAngleBetweenPlanes(armPlane, upperArmPlane, inRadians);
            }
            else
            {
                angle = Plane.GetAngleBetweenPlanes(armPlane, upperArmPlane, inRadians);
            }

            double distFromPlane = wrist.DistanceFromPlane(upperArmPlane);
            if (distFromPlane < 0)
            {
                return -angle;
            }
            return angle;
        }


        /// <summary>
        /// Calculates the shoulder roll for the left arm.
        /// </summary>
        /// <param name="neckToSpine">The vector from the neck point to the spine point.</param>
        /// <param name="shoulderR2L">The vector from right to left shoulder.</param>
        /// <param name="shoulderToElbow">The vector from the left shoulder to the left elbow.</param>
        /// <param name="elbowToWrist">The vector from the left elbow to the left wrist.</param>
        /// <param name="inRadians">Whether to get the angle in radians instead of degrees. Defaults to false.</param>
        /// <returns>The angle of the shoulder roll.</returns>
        public static double GetShoulderRollLeft(Vector3 neckToSpine, Vector3 shoulderR2L, Vector3 shoulderToElbow, Vector3 elbowToWrist, double forwardFacingRatio, bool inRadians = false)
        {
            // 1. Get Body plane 1 (neck-spine and shoulder-shoulder).
            // 2. Get Body plane 2 (shoulder-shoulder and Body Plane 1 perpendicular).
            // 3. Get Arm plane (shoulder-elbow and elbow-wrist).
            // 4. Get angle of planes.
            // 5. Adapt angle to be in the range of 90 deg (pointing up) to -90
            // (pointing down).

            Plane bodyPlane1 = Plane.FromVectors(neckToSpine, shoulderR2L);
            Plane bodyPlane2 = Plane.FromVectors(bodyPlane1.AsVector(), shoulderR2L);
            Plane armPlane = Plane.FromVectors(shoulderToElbow, elbowToWrist);

            if (armPlane.AsVector().IsEmpty)
            {
                return 0;
            }

            // For when elbow is pointing to the side or down, in the range of
            // 90 (pointing up) and -90 (pointing down). We need to use the
            // opposite direction of the plane perpendicular. This gives the
            // right polarity of the angles. In addition, we need to subtract
            // the initial perpendicularity of the arm plane.
            Vector3 bodyPlane1Perpendicular = bodyPlane1.AsVector();
            double angleRelativeToBodyPerpendicular = Vector3.GetAngleBetweenVectors(bodyPlane1Perpendicular * -1, armPlane.AsVector(), inRadians);
            angleRelativeToBodyPerpendicular = Double.IsNaN(angleRelativeToBodyPerpendicular)
                ? _90Deg(inRadians)
                : angleRelativeToBodyPerpendicular - _90Deg(inRadians);

            // For when the elbow is pointing forward, in the range of 90
            // (pointing up) and -90 (pointing down).
            double angleRelativeToShoulders = Vector3.GetAngleBetweenVectors(bodyPlane2.AsVector() * -1, armPlane.AsVector(), inRadians);

            if (Double.IsNaN(angleRelativeToShoulders))
            {
                // When the arm lines up, set to the reset position.
                angleRelativeToShoulders = _90Deg(inRadians);
            }

            if (forwardFacingRatio > 1)
            {
                forwardFacingRatio = 1;
            }
            else if (forwardFacingRatio < -1)
            {
                forwardFacingRatio = -1;
            }

            return ((1 - forwardFacingRatio) * angleRelativeToBodyPerpendicular) + (forwardFacingRatio * angleRelativeToShoulders);
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
            // For forward kinematics, this needs to be from 0 (extended) to
            // -180 (overly-contracted).
            return -Vector3.GetAngleBetweenVectors(shoulderToElbow, elbowToWrist, inRadians);
        }

        private static double _90Deg(bool inRadians)
        {
            return inRadians ? 90 * deg2rad : 90;
        }
    }
}

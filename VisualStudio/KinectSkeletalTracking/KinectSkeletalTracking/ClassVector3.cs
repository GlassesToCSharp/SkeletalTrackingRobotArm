using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KinectSkeletalTracking
{
    public class Vector3
    {
        private double x = 0.0;
        private double y = 0.0;
        private double z = 0.0;
        private double magnitude = 0.0;

        public double X
        {
            get { return x; }
            set
            {
                x = value;
                magnitude = Math.Sqrt((x * x) + (y * y) + (z * z));
            }
        }
        public double Y
        {
            get { return y; }
            set
            {
                y = value;
                magnitude = Math.Sqrt((x * x) + (y * y) + (z * z));
            }
        }
        public double Z
        {
            get { return z; }
            set
            {
                z = value;
                magnitude = Math.Sqrt((x * x) + (y * y) + (z * z));
            }
        }

        public double Magnitude
        {
            get
            {
                return (this is Plane ? Double.NaN : magnitude);
            }
        }
    }

    /// <summary>
    /// Gives vector perpendicular to plane.
    /// Should return in the format Xx + Yy + Zz + D = 0
    /// </summary>
    public class Plane : Vector3
    {
        public double D { get; set; } = 0.0;
    }
}

using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace KinectSkeletalTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private const bool SendToMotors = true;

        private const double Rad2Deg = 180 / Math.PI;
        private const double Deg2Rad = Math.PI / 180;

        private const string SerialDataFormat = "S{0}E";

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;


        SolidColorBrush[] colours = new SolidColorBrush[]
        {
            Brushes.Red,
            Brushes.Orange,
            Brushes.Green,
            Brushes.Blue,
            Brushes.Indigo,
            Brushes.Violet
        };

        enum ColoursIndex
        {
            Red, Orange, Green, Blue, Indigo, Violet
        }

        private List<string> bodiesList = new List<string>();
        public List<string> BodiesList
        {
            get { return bodiesList; }
            set
            {
                bodiesList = value;
                OnPropertyChanged(nameof(BodiesList));

                if(bodiesListIndex > bodiesList.Count)
                {
                    bodiesListIndex = bodiesList.Count - 1;
                }
            }
        }

        private int bodiesListIndex = 0;
        public int BodiesListIndex
        {
            get { return bodiesListIndex; }
            set
            {
                bodiesListIndex = value;
                OnPropertyChanged(nameof(bodiesListIndex));
            }
        }

        private Brush textColourOfBody = new SolidColorBrush(Color.FromRgb(0, 0, 0));
        public Brush TextColourOfBody
        {
            get { return textColourOfBody; }
            set
            {
                textColourOfBody = value;
                OnPropertyChanged(nameof(TextColourOfBody));
            }
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();
            
            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(colours[(int)ColoursIndex.Red], 6));
            this.bodyColors.Add(new Pen(colours[(int)ColoursIndex.Orange], 6));
            this.bodyColors.Add(new Pen(colours[(int)ColoursIndex.Green], 6));
            this.bodyColors.Add(new Pen(colours[(int)ColoursIndex.Blue], 6));
            this.bodyColors.Add(new Pen(colours[(int)ColoursIndex.Indigo], 6));
            this.bodyColors.Add(new Pen(colours[(int)ColoursIndex.Violet], 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            if (SendToMotors)
                SerialInterface.Instance.Init();

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource => this.imageSource;

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get { return statusText; }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    OnPropertyChanged(nameof(StatusText));
                }
            }
        }

        private void OnPropertyChanged(string nameOfProperty)
        {
            // notify any bound elements that the property has changed
            this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameOfProperty));
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    List<string> tempListstring = new List<string>();

                    int penIndex = 0;
                    for (int bodyIndex = 0; bodyIndex < bodies.Count(); bodyIndex++)
                    //foreach (Body body in this.bodies)
                    {
                        Body body = bodies[bodyIndex];

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            tempListstring.Add($"{penIndex - 1} - {(ColoursIndex)(penIndex - 1)}");

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                // At this point, calculate the angles of the arm joints relative to the shoulder plane.

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }
                            
                            string selection = String.Empty;
                            int selectedIndex = 0;
                            if (BodiesList.Count > 0 &&
                                BodiesListIndex >= 0 &&
                                !String.IsNullOrWhiteSpace(selection = BodiesList[BodiesListIndex]) &&
                                Int32.TryParse(selection.Substring(0, 1), out selectedIndex) &&
                                selectedIndex == bodyIndex)
                            {
                                TextColourOfBody = drawPen.Brush;
                                // for Testing, get elbow angle by doing shoulder->elbow and wrist->elbow
                                //TestElbowPitch(ref joints);
                                //TestShoulderPitch(ref joints);
                                TestShoulderToElbow(ref joints);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    BodiesList = tempListstring;

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }


        #region Testing stuff

        /// <summary>
        /// Gets the angle of the elbow joint from the upper arm and lower arm vectors.
        /// </summary>
        /// <param name="joints">List of joint positions.</param>
        private void TestElbowPitch(ref IReadOnlyDictionary<JointType, Joint> joints)
        {
            // 1. Get vectors from shoulder -> elbow
            CameraSpacePoint pointForShoulder = joints[JointType.ShoulderRight].Position;
            CameraSpacePoint pointForElbow = joints[JointType.ElbowRight].Position;
            Vector3 shoulderToElbow = GetVectorFromPoints(pointForElbow, pointForShoulder);

            // 2. Get vectors from wrist -> elbow
            CameraSpacePoint pointForWrist = joints[JointType.WristRight].Position;
            Vector3 wristToElbow = GetVectorFromPoints(pointForElbow, pointForWrist);

            // 3. Get angle between vectors
            double elbowAngle = GetAngleBetweenVectors(shoulderToElbow, wristToElbow);

            // 4. Send this angle to motors
            SerialWrite(elbowAngle);
        }


        /// <summary>
        /// Gets the angle of the shoulder joint pitch from the upper arm vector and left->right shoulder vector.
        /// </summary>
        /// <param name="joints">List of joint positions.</param>
        private void TestShoulderPitch(ref IReadOnlyDictionary<JointType, Joint> joints)
        {
            // 1. Get vectors from elbow -> shoulderRight
            CameraSpacePoint pointForShoulderRight = joints[JointType.ShoulderRight].Position;
            CameraSpacePoint pointForElbow = joints[JointType.ElbowRight].Position;
            Vector3 shoulderRightToElbow = GetVectorFromPoints(pointForElbow, pointForShoulderRight);

            // 2. Get vectors from shoulderLeft -> shoulderRight
            CameraSpacePoint pointForShoulderLeft = joints[JointType.ShoulderLeft].Position;
            Vector3 shoulderLeftToShoulderRight = GetVectorFromPoints(pointForShoulderLeft, pointForShoulderRight);

            // 3. Get angle between vectors
            double elbowAngle = GetAngleBetweenVectors(shoulderLeftToShoulderRight, shoulderRightToElbow);

            // 4. Send this angle to motors
            SerialWrite(elbowAngle);
        }


        /// <summary>
        /// Gets the angle of the shoulder joint pitch and yaw.
        /// </summary>
        /// <param name="joints">List of joint positions.</param>
        private void TestShoulderToElbow(ref IReadOnlyDictionary<JointType, Joint> joints)
        {
            // 1. Get elbow and shoulder points
            CameraSpacePoint pointForShoulderRight = joints[JointType.ShoulderRight].Position;
            CameraSpacePoint pointForShoulderleft = joints[JointType.ShoulderLeft].Position;
            CameraSpacePoint pointForElbow = joints[JointType.ElbowRight].Position;
            CameraSpacePoint pointForWrist = joints[JointType.WristRight].Position;

            // 2. Created necessary planes and vectors
            Plane bodyPlane = GetPlaneFromPoints(joints[JointType.ShoulderRight].Position, joints[JointType.Neck].Position, joints[JointType.SpineShoulder].Position);
            Vector3 spineToNeck = new Vector3()
            {
                X = bodyPlane.X,
                Y = bodyPlane.Y,
                Z = bodyPlane.Z
            };
            Vector3 crossShoulderVector = GetVectorFromPoints(pointForShoulderRight, pointForShoulderleft);
            Vector3 shoulderToElbow = GetVectorFromPoints(pointForShoulderRight, pointForElbow);
            Vector3 shoulderToWrist = GetVectorFromPoints(pointForShoulderRight, pointForWrist);
            Vector3 wristToElbow = GetVectorFromPoints(pointForElbow, pointForWrist);

            // 3 Calculate the anngles of each degree of freedom, here
            //   theta1 = shoulder yaw
            //   theta2 = shoulder pitch
            //   theta3 = shoulder roll
            //   theta4 = elbow pitch
            double theta1 = GetAngleFromSpineToElbow(ref spineToNeck, ref shoulderToElbow);
            double theta2 = GetAngleFromCrossShoulderToElbow(ref crossShoulderVector, ref shoulderToElbow);
            double theta3 = GetShoulderRoll(ref bodyPlane, ref shoulderToElbow, ref shoulderToWrist);
            double theta4 = GetElbowAngle(ref shoulderToElbow, ref wristToElbow);
            
            // 3.5 Display the angles on the screen
            Angles.Text = String.Format("Shoulder {0}, {1}, {2}" + Environment.NewLine +
                "Elbow {3}",
                (int)(theta1), (int)(theta2), (int)(theta3),
                (int)(theta4));

            // 4. Send this angle to motors
            SerialWrite(new double[] 
            {
                theta1,
                theta2,
                theta3,
                theta4
            });
        }

        #endregion

        #region Serial stuff

        private void SerialWrite(double angle)
        {
            SerialWrite(new double[1] { angle });
        }

        private void SerialWrite(params double[] angles)
        {
            string dataToSend = ",";
            foreach(double angle in angles)
            {
                dataToSend += (int)(angle) + ",";
            }

            if (SendToMotors && SerialInterface.Instance.CanWrite)
            {
                SerialInterface.Instance.Write(String.Format(SerialDataFormat, dataToSend));
            }
        }

        #endregion

        #region Math stuff

        /// <summary>
        /// Calculates the shoulder pitch
        /// </summary>
        private double GetAngleFromCrossShoulderToElbow(ref Vector3 crossShoulder, ref Vector3 shoulderElbow)
        {
            return GetAngleBetweenVectors(crossShoulder, shoulderElbow) * Rad2Deg;
        }


        /// <summary>
        /// Calculates the shoulder yaw.
        /// </summary>
        private double GetAngleFromSpineToElbow(ref Vector3 spine, ref Vector3 shoulderElbow)
        {
            return GetAngleBetweenVectors(spine, shoulderElbow) * Rad2Deg;
        }


        /// <summary>
        /// Calculates the shoulder roll.
        /// </summary>
        private double GetShoulderRoll(ref Plane body, ref Vector3 shoulderToElbow, ref Vector3 elbowToWrist)
        {
            // 1. Get shoulder-elbow-wrist plane
            Plane armPlane = GetPlaneFromVectors(new Vector3[]
            {
                shoulderToElbow * -1,
                elbowToWrist
            });

            // 2. Get orthogonal vector of Body plane
            Vector3 bodyOrthogonal = new Vector3
            {
                X = body.X,
                Y = body.Y,
                Z = body.Z
            };

            // 3. Get elbow-shoulder / elbow-wrist orthogonal vector
            Vector3 armOrthogonal = new Vector3
            {
                X = armPlane.X,
                Y = armPlane.Y,
                Z = armPlane.Z
            };

            // 4. Calculate angle between orthogonal vector of Body and orthogonal vector of Arm
            return GetAngleBetweenVectors(bodyOrthogonal, armOrthogonal) * Rad2Deg;
        }


        /// <summary>
        /// Calculates the elbow angle.
        /// </summary>
        private double GetElbowAngle(ref Vector3 shoulderToElbow, ref Vector3 wristToElbow)
        {
            return GetAngleBetweenVectors(shoulderToElbow, wristToElbow) * Rad2Deg;
        }


        private Vector3 GetVectorFromPoints(CameraSpacePoint p, CameraSpacePoint q)
        {
            if (p.Z < 0)
            {
                p.Z = InferredZPositionClamp;
            }
            if (q.Z < 0)
            {
                q.Z = InferredZPositionClamp;
            }

            return new Vector3
            {
                X = q.X - p.X,
                Y = q.Y - p.Y,
                Z = q.Z - p.Z
            };
        }


        private double GetAngleBetweenVectors(Vector3 u, Vector3 v)
        {
            return Vector3.GetAngleBetweenVectors(u, v);
        }


        private Plane GetPlaneFromPoints(params CameraSpacePoint[] points)
        {
            // minimum 3 points are required
            if (points.Length < 3)
            {
                return null;
            }

            // Only 3 points are needed to make a plane
            CameraSpacePoint A = points[0];
            CameraSpacePoint B = points[1];
            CameraSpacePoint C = points[2];

            Vector3 AB = GetVectorFromPoints(A, B);
            Vector3 AC = GetVectorFromPoints(A, C);

            // Do the cross product
            Vector3 normal = AB.Cross(AC);

            // Return the normal with the ammended D value. This is the Plane equation.
            return new Plane
            {
                X = normal.X,
                Y = normal.Y,
                Z = normal.Z,
                D = -((normal.X * (-A.X)) + (normal.Y * (-A.Y)) + (normal.Z * (-A.Z)))
            };
        }


        private Plane GetPlaneFromVectors(params Vector3[] vectors)
        {
            // minimum 2 vectors are required
            if (vectors.Length < 2)
            {
                return null;
            }

            // Only 2 vectors are needed to make a plane
            Vector3 u = vectors[0];
            Vector3 v = vectors[1];

            // Do the cross product
            Vector3 orthogonal = u.Cross(v);

            // Return the normal with the ammended D value. This is the Plane equation.
            return new Plane
            {
                X = orthogonal.X,
                Y = orthogonal.Y,
                Z = orthogonal.Z,
                D = -((orthogonal.X * (-u.X)) + (orthogonal.Y * (-u.Y)) + (orthogonal.Z * (-u.Z)))
            };
        }

        #endregion
    }
}

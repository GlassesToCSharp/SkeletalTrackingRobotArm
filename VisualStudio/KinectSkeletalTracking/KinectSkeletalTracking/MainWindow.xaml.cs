using Matrix = MatrixDesign.Matrix;
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Controls;

namespace KinectSkeletalTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private const bool SendToMotors = false;

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
        private readonly DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing group for body projection output
        /// </summary>
        private readonly DrawingGroup projectedGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private readonly DrawingImage imageSource;

        /// <summary>
        /// Projected image that we will display
        /// </summary>
        private readonly DrawingImage projectedSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private readonly CoordinateMapper coordinateMapper = null;

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
        private readonly List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private readonly int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private readonly int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private readonly List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;


        readonly SolidColorBrush[] colours = new SolidColorBrush[]
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

                if (bodiesListIndex > bodiesList.Count)
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
            this.bones = new List<Tuple<JointType, JointType>>
            {
                // Torso
                new Tuple<JointType, JointType>(JointType.Head, JointType.Neck),
                new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder),
                new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid),
                new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase),
                new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight),
                new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft),
                new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight),
                new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft),

                // Right Arm
                new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight),
                new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight),
                new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight),
                new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight),
                new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight),

                // Left Arm
                new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft),
                new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft),
                new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft),
                new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft),
                new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft),

                // Right Leg
                new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight),
                new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight),
                new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight),

                // Left Leg
                new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft),
                new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft),
                new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft)
            };

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>
            {
                new Pen(colours[(int)ColoursIndex.Red], 6),
                new Pen(colours[(int)ColoursIndex.Orange], 6),
                new Pen(colours[(int)ColoursIndex.Green], 6),
                new Pen(colours[(int)ColoursIndex.Blue], 6),
                new Pen(colours[(int)ColoursIndex.Indigo], 6),
                new Pen(colours[(int)ColoursIndex.Violet], 6)
            };

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create the drawing group we'll use for projecting
            this.projectedGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Create the projected image we need to draw.
            this.projectedSource = new DrawingImage(this.projectedGroup);

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
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ProjectionSource => this.projectedSource;

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

                    for (int bodyIndex = 0; bodyIndex < bodies.Count(); bodyIndex++)
                    //foreach (Body body in this.bodies)
                    {
                        Body body = bodies[bodyIndex];

                        Pen drawPen = bodyColors[3];// this.bodyColors[bodyIndex];

                        if (body.IsTracked)
                        {
                            tempListstring.Add($"{bodyIndex} - {(ColoursIndex)(bodyIndex)}");

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // Get the 2D representation of the depth point.
                                DepthSpacePoint depthSpacePoint = this.GetDepthSpacePoint(joints[jointType].Position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            string selection;
                            if (BodiesList.Count > 0 &&
                                BodiesListIndex >= 0 &&
                                !String.IsNullOrWhiteSpace(selection = BodiesList[BodiesListIndex]) &&
                                Int32.TryParse(selection.Substring(0, 1), out int selectedIndex) &&
                                selectedIndex == bodyIndex)
                            {
                                // At this point, calculate the angles of the arm joints relative to the shoulder plane.
                                FollowBody(drawPen, ref joints);
                            }


                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    BodiesList = tempListstring;

                    //string selection = "";
                    //int selectedIndex = 0;

                    // If there is only one body being tracked, automatically
                    // follow that one. Otherwise, follow the selected body.
                    //if ((BodiesList.Count == 1) || (BodiesList.Count > 0 &&
                    //    BodiesListIndex >= 0 &&
                    //    !String.IsNullOrWhiteSpace(selection) &&
                    //    Int32.TryParse(selection.Substring(0, 1), out selectedIndex) &&
                    //    selectedIndex < bodies.Length))
                    //System.Diagnostics.Debug.WriteLine($"BodiesListCount: {BodiesList.Count}");
                    //System.Diagnostics.Debug.WriteLine($"bodyColours count: {bodyColors.Count}");
                    //System.Diagnostics.Debug.WriteLine($"bodies count: {bodies.Length}");
                    //foreach (string colour in BodiesList)
                    //{
                    //    System.Diagnostics.Debug.WriteLine($"Colour: {colour}");
                    //}
                    //if (BodiesList.Count == 1)
                    //{
                    //    Pen drawPen = bodyColors[selectedIndex];
                    //    IReadOnlyDictionary<JointType, Joint> joints = bodies[selectedIndex].Joints;
                    //    FollowBody(drawPen, ref joints);
                    //}

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
            if (!joints.ContainsKey(jointType0) || !joints.ContainsKey(jointType1))
            {
                // If the joints aren't being drawn, don't draw the bones for them.
                return;
            }

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
        /// Sometimes the depth(Z) of an inferred joint may show as negative.
        /// Clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
        /// </summary>
        /// <param name="position">position of the joint</param>
        /// <returns>The 2D representation of the position</returns>
        private DepthSpacePoint GetDepthSpacePoint(CameraSpacePoint position)
        {
            if (position.Z < 0)
            {
                position.Z = InferredZPositionClamp;
            }

            return this.coordinateMapper.MapCameraPointToDepthSpace(position);
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

        private void FollowBody(Pen drawPen, ref IReadOnlyDictionary<JointType, Joint> joints)
        {
            TextColourOfBody = drawPen.Brush;
            // for Testing, get elbow angle by doing shoulder->elbow and wrist->elbow
            //TestElbowPitch(ref joints);
            //TestShoulderPitch(ref joints);
            TestShoulderToElbow(ref joints);
        }

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
            double elbowAngle = Vector3.GetAngleBetweenVectors(shoulderToElbow, wristToElbow);

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
            double elbowAngle = Vector3.GetAngleBetweenVectors(shoulderLeftToShoulderRight, shoulderRightToElbow);

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
            CameraSpacePoint pointForShoulderLeft = joints[JointType.ShoulderLeft].Position;
            CameraSpacePoint pointForElbow = joints[JointType.ElbowRight].Position;
            CameraSpacePoint pointForWrist = joints[JointType.WristRight].Position;

            // 2. Created necessary planes and vectors
            Plane bodyPlane = Plane.FromPoints(joints[JointType.ShoulderRight].Position, joints[JointType.Neck].Position, joints[JointType.SpineShoulder].Position);
            Vector3 spineToNeck = new Vector3()
            {
                X = bodyPlane.X,
                Y = bodyPlane.Y,
                Z = bodyPlane.Z
            };
            Vector3 crossShoulderVector = Vector3.FromPoints(pointForShoulderRight, pointForShoulderLeft);
            Vector3 shoulderToElbow = Vector3.FromPoints(pointForShoulderRight, pointForElbow);
            Vector3 shoulderToWrist = Vector3.FromPoints(pointForShoulderRight, pointForWrist);
            Vector3 elbowToWrist = Vector3.FromPoints(pointForElbow, pointForWrist);

            // 3. Calculate the anngles of each degree of freedom, here
            //    theta1 = shoulder yaw
            //    theta2 = shoulder pitch
            //    theta3 = shoulder roll
            //    theta4 = elbow pitch
            double shoulderYawAngleDegrees = GetAngleFromSpineToElbow(ref spineToNeck, ref shoulderToElbow);
            double shoulderPitchAngleDegrees = GetAngleFromCrossShoulderToElbow(ref crossShoulderVector, ref shoulderToElbow);
            double shoulderRollAngleDegrees = GetShoulderRoll(ref bodyPlane, ref shoulderToElbow, ref shoulderToWrist);
            double elbowPitchAngleDegrees = GetElbowAngle(ref shoulderToElbow, ref wristToElbow);

            // 4. Display the angles on the screen
            using (DrawingContext dc = this.projectedGroup.Open())
            {
                // Using matrices and forward kinematics.

                // Base - right shoulder point.
                // T01 - Translation from base to shoulder yaw (revolution about X axis)
                // T12 - Translation from shoulder yaw to shoulder pitch (revolution about Z axis)
                // T23 - Translation from shoulder pitch to elbow joint (translation through X axis)
                // T03 - Translation from base to elbow joint
                //  L1 - Length from shoulder to elbow

                double l1 = shoulderToElbow.Magnitude;
                double theta1 = shoulderYawAngleDegrees * Deg2Rad;
                double theta2 = shoulderPitchAngleDegrees * Deg2Rad;
                double theta3 = shoulderRollAngleDegrees * Deg2Rad;
                double theta4 = elbowPitchAngleDegrees * Deg2Rad;

                Matrix t01 = new Matrix(4, 4, new double[,] {
                    { 1, 0, 0, 0 },
                    { 0, Math.Cos(theta1), -Math.Sin(theta1), 0 },
                    { 0, Math.Sin(theta1), Math.Cos(theta1), 0 },
                    { 0, 0, 0, 1 },
                });
                Matrix t12 = new Matrix(4, 4, new double[,] {
                    { Math.Cos(theta2), -Math.Sin(theta2), 0, 0 },
                    { Math.Sin(theta2), Math.Cos(theta2), 0, 0 },
                    { 0, 0, 1, 0 },
                    { 0, 0, 0, 1 },
                });
                Matrix t23 = new Matrix(4, 4, new double[,] {
                    { 1, 0, 0, l1 },
                    { 0, 1, 0, 0 },
                    { 0, 0, 1, 0 },
                    { 0, 0, 0, 1 },
                });

                Matrix t03 = t01 * t12 * t23;
                Vector3 elbowPosition = GetPositionVectorFromMatrix(t03);


                // Base - elbow point.
                // T34 - Translation from base to shoulder roll (revolution about X axis)
                // T45 - Translation from shoulder roll to elbow pitch (revolution about Y axis)
                // T5e - Translation from elbow pitch to end effector (translation through X axis)
                // T3e - Translation from base to end effector
                //  L2 - Length from elbow to end effector

                double l2 = elbowToWrist.Magnitude;

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
                Vector3 wristPosition = GetPositionVectorFromMatrix(t0e);
                wristPosition -= elbowPosition;


                // Draw the skeleton (neck/spine and shoulders)
                // Draw the projected vectors

                Pen drawingPen = bodyColors[2]; // new Pen(new SolidColorBrush(Color.FromArgb(255, 68, 192, 68)), 3);
                Pen skeletonPen = this.inferredBonePen;

                // Use the angles to create projected vectors of joints. Use Forward Kinematics.
                IReadOnlyDictionary<JointType, Joint> armPoints = new Dictionary<JointType, Joint>
                {
                    [JointType.ShoulderLeft] = new Joint()
                    {
                        JointType = JointType.ShoulderLeft,
                        Position = pointForShoulderLeft,
                        TrackingState = TrackingState.Tracked,
                    },
                    [JointType.ShoulderRight] = new Joint()
                    {
                        JointType = JointType.ShoulderRight,
                        Position = pointForShoulderRight,
                        TrackingState = TrackingState.Tracked,
                    },
                    [JointType.ElbowRight] = new Joint()
                    {
                        JointType = JointType.ElbowRight,
                        Position = elbowPosition.ToCameraSpacePoint(),
                        TrackingState = TrackingState.Tracked,
                    },
                    [JointType.WristRight] = new Joint()
                    {
                        JointType = JointType.WristRight,
                        Position = wristPosition.ToCameraSpacePoint(),
                        TrackingState = TrackingState.Tracked,
                    }
                };

                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                // convert the joint points to depth (display) space
                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                foreach (JointType jointType in armPoints.Keys)
                {
                    // Get the 2D representation of the depth point.
                    DepthSpacePoint depthSpacePoint = this.GetDepthSpacePoint(armPoints[jointType].Position);
                    jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                }


                this.DrawBody(new Dictionary<JointType, Joint>
                {
                    [JointType.ShoulderLeft] = armPoints[JointType.ShoulderLeft],
                    [JointType.ShoulderRight] = armPoints[JointType.ShoulderRight]
                }, jointPoints, dc, inferredBonePen);
                this.DrawBody(new Dictionary<JointType, Joint>
                {
                    [JointType.ShoulderRight] = armPoints[JointType.ShoulderRight],
                    [JointType.ElbowRight] = armPoints[JointType.ElbowRight]
                }, jointPoints, dc, bodyColors[4]);
                this.DrawBody(new Dictionary<JointType, Joint>
                {
                    [JointType.ElbowRight] = armPoints[JointType.ElbowRight],
                    [JointType.WristRight] = armPoints[JointType.WristRight],
                }, jointPoints, dc, bodyColors[1]);


                // prevent drawing outside of our render area
                this.projectedGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                Angles.Text = String.Format(
                    "Shoulder " + SpacePointToString(armPoints[JointType.ShoulderRight].Position) + Environment.NewLine +
                    "Elbow    " + SpacePointToString(armPoints[JointType.ElbowRight].Position) + Environment.NewLine +
                    "Wrist    " + SpacePointToString(armPoints[JointType.WristRight].Position) + Environment.NewLine +
                    "U Arm L  " + l1 + Environment.NewLine +
                    "L Arm L  " + l2 + Environment.NewLine +
                    "Shoulder {0}, {1}, {2}" + Environment.NewLine +
                    "Elbow {3}",
                    (int)shoulderYawAngleDegrees, (int)shoulderPitchAngleDegrees, (int)shoulderRollAngleDegrees,
                    (int)elbowPitchAngleDegrees);
            }

            //Angles.Text = String.Format("Shoulder {0}, {1}, {2}" + Environment.NewLine +
            //    "Elbow {3}",
            //    (int)shoulderYawAngleDegrees, (int)shoulderPitchAngleDegrees, (int)shoulderRollAngleDegrees,
            //    (int)elbowPitchAngleDegrees);

            // 5. Send this angle to motors
            SerialWrite(FilterAngles(new double[]
            {
                shoulderYawAngleDegrees,
                shoulderPitchAngleDegrees,
                shoulderRollAngleDegrees,
                elbowPitchAngleDegrees
            }));
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
            foreach (double angle in angles)
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
            return -Vector3.GetAngleBetweenVectors(crossShoulder, shoulderElbow);
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
            return Vector3.GetAngleBetweenVectors(bodyOrthogonal, armOrthogonal);
        }


        /// <summary>
        /// Calculates the elbow angle.
        /// </summary>
        private double GetElbowAngle(ref Vector3 shoulderToElbow, ref Vector3 wristToElbow)
        {
            return Vector3.GetAngleBetweenVectors(shoulderToElbow, wristToElbow);
        }


        private Vector3 GetVectorFromPoints(CameraSpacePoint p, CameraSpacePoint q)
        {
            return Vector3.FromPoints(p, q, InferredZPositionClamp);
        }


        private Vector3 GetPositionVectorFromMatrix(Matrix matrix)
        {
            if (matrix.Rows < 3 || matrix.Columns < 3)
            {
                throw new Exception("Not enough columns or rows.");
            }

            return new Vector3
            {
                X = matrix.GetRow(0).Last(),
                Y = matrix.GetRow(1).Last(),
                Z = matrix.GetRow(2).Last()
            };
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

        #region Filtering

        private const byte averageSpan = 5;
        private const byte columns = 4;
        private double[,] recentAngles;
        /// <summary>
        /// Applies a moving average filter to the angles. The new values are
        /// averaged with the previous 4 (four), giving an overall average
        /// across 5 (five) values.
        /// </summary>
        /// <param name="angles"></param>
        /// <returns></returns>
        double[] FilterAngles(double[] angles)
        {
            if (recentAngles == null)
            {
                recentAngles = new double[averageSpan, columns]; // 4 angle values for now.
                for (byte i = 0; i < averageSpan; i++)
                {
                    for (byte j = 0; j < columns; j++)
                    {
                        recentAngles[i, j] = angles[j];
                    }

                }
                // The average of all the same angles is going to be the
                // assigned angles.
                return angles;
            }

            // Get the average of the angles
            double[] averagedAngles = new double[columns];
            for (byte i = 0; i < columns; i++)
            {
                double sum = 0;
                // For calulating the average, we are not interested in the
                // oldest value. We will need to add the new values instead.
                for (byte j = 1; j < averageSpan; j++)
                {
                    double currentValue = recentAngles[j, i];
                    sum += currentValue;

                    recentAngles[j - 1, i] = currentValue;
                }

                recentAngles[averageSpan - 1, i] = angles[i];
                sum += angles[i];

                averagedAngles[i] = sum / averageSpan;
            }
            //System.Diagnostics.Debug.WriteLine(String.Format("Averaged Angles: {0}, {1}, {2}, {3}", averagedAngles[0], averagedAngles[1], averagedAngles[2], averagedAngles[3]));
            return averagedAngles;
        }

        #endregion
    }
}

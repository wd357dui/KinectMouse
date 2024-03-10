using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Forms;

namespace KinectMouse
{
	public partial class KinectMouse : Form
	{
		public KinectMouse()
		{
			InitializeComponent();
		}

		KinectSensor sensor;
		BodyFrameReader reader;
		private void KinectMouse_Load(object sender, EventArgs e)
		{
			sensor = KinectSensor.GetDefault();
			reader = sensor.BodyFrameSource.OpenReader();
			reader.FrameArrived += Reader_FrameArrived;
			sensor.Open();
		}

		Body[] bodies = null;
		readonly Queue<Joint> headBuffer = new Queue<Joint>();
		readonly Queue<Joint> pelvisBuffer = new Queue<Joint>();
		readonly Queue<Joint> leftHandBuffer = new Queue<Joint>();
		readonly Queue<Joint> rightHandBuffer = new Queue<Joint>();
		readonly Queue<TrackingConfidence> leftHandConfidence = new Queue<TrackingConfidence>();
		readonly Queue<TrackingConfidence> rightHandConfidence = new Queue<TrackingConfidence>();
		readonly Queue<HandState> leftHandState = new Queue<HandState>();
		readonly Queue<HandState> rightHandState = new Queue<HandState>();
		const int bufferCount = 8;
		const int activeThreshold = 3;
		const int lassoThreshold = 8;
		const decimal highConfidenceThreshold = 0.5m;
		const float sensitivityX = 1.0f;
		const float sensitivityY = 1.5f;
		bool isMoving = false;
		bool isDown = false;
		bool isLasso = false;
		private bool IsCloserToHead(Joint head, Joint pelvis, Joint hand)
		{
			float DistanceToHeadX = Math.Abs(hand.Position.X - head.Position.X);
			float DistanceToHeadY = Math.Abs(hand.Position.Y - head.Position.Y);
			float DistanceToPelvisX = Math.Abs(hand.Position.X - pelvis.Position.X);
			float DistanceToPelvisY = Math.Abs(hand.Position.Y - pelvis.Position.Y);
			return DistanceToHeadX + DistanceToHeadY < DistanceToPelvisX + DistanceToPelvisY;
		}
		private bool IsCloserToHead(Joint[] head, Joint[] pelvis, Joint[] hand)
		{
			int length = Math.Min(Math.Min(hand.Length, head.Length), pelvis.Length);
			for (int i = 0; i < length; i++)
			{
				if (!IsCloserToHead(head[i], pelvis[i], hand[i])) return false;
			}
			return true;
		}
		private bool TestHandClosed(ref bool closed, Queue<Joint> handBuffer, Queue<TrackingConfidence> confidence, Queue<HandState> state)
		{
			if (!closed)
			{
				// test several high confidence frames of "closed" state to enter moving state
				if (headBuffer.Count >= activeThreshold &&
					pelvisBuffer.Count >= activeThreshold &&
					handBuffer.Count >= activeThreshold &&
					confidence.Count >= activeThreshold &&
					state.Count >= activeThreshold &&
					state.Reverse().Take(activeThreshold).All(S => S == HandState.Closed) &&
					decimal.Divide(confidence.Reverse().Take(activeThreshold).Sum(C => C == TrackingConfidence.High ? 1 : 0), activeThreshold) >= highConfidenceThreshold &&
					IsCloserToHead(
						headBuffer.Reverse().Take(activeThreshold).ToArray(),
						pelvisBuffer.Reverse().Take(activeThreshold).ToArray(),
						handBuffer.Reverse().Take(activeThreshold).ToArray()))
				{
					closed = true;
				}
			}
			else
			{
				// test several high confidence frames of non closed state or non tracked state to exit moving state
				if (handBuffer.Count >= activeThreshold &&
					confidence.Count >= activeThreshold &&
					state.Count >= activeThreshold &&
					handBuffer.Reverse().Take(activeThreshold).All(J => J.TrackingState == TrackingState.Tracked) &&
					decimal.Divide(confidence.Reverse().Take(activeThreshold).Sum(C => C == TrackingConfidence.High ? 1 : 0), activeThreshold) >= highConfidenceThreshold &&
					state.Reverse().Take(activeThreshold).All(S => S != HandState.Closed && S != HandState.Unknown && S != HandState.NotTracked)
					||
					handBuffer.Reverse().Take(activeThreshold).All(J => J.TrackingState == TrackingState.NotTracked) &&
					state.Reverse().Take(activeThreshold).All(S => S == HandState.NotTracked))
				{
					closed = false;
				}
			}
			return closed;
		}
		private bool TestHandLasso(ref bool isLasso, Queue<Joint> handBuffer, Queue<TrackingConfidence> confidence, Queue<HandState> state)
		{
			if (!isLasso)
			{
				// test several high confidence frames of "lasso" state to enter moving state
				if (headBuffer.Count >= lassoThreshold &&
					pelvisBuffer.Count >= lassoThreshold &&
					handBuffer.Count >= lassoThreshold &&
					confidence.Count >= lassoThreshold &&
					state.Count >= lassoThreshold &&
					state.Reverse().Take(lassoThreshold).All(S => S == HandState.Lasso) &&
					decimal.Divide(confidence.Reverse().Take(lassoThreshold).Sum(C => C == TrackingConfidence.High ? 1 : 0), lassoThreshold) >= highConfidenceThreshold &&
					IsCloserToHead(
						headBuffer.Reverse().Take(lassoThreshold).ToArray(),
						pelvisBuffer.Reverse().Take(lassoThreshold).ToArray(),
						handBuffer.Reverse().Take(lassoThreshold).ToArray()))
				{
					isLasso = true;
				}
			}
			else
			{
				// test several high confidence frames of non lasso state or non tracked state to exit moving state
				if (handBuffer.Count >= activeThreshold &&
					confidence.Count >= activeThreshold &&
					state.Count >= activeThreshold &&
					handBuffer.Reverse().Take(activeThreshold).All(J => J.TrackingState == TrackingState.Tracked) &&
					decimal.Divide(confidence.Reverse().Take(activeThreshold).Sum(C => C == TrackingConfidence.High ? 1 : 0), activeThreshold) >= highConfidenceThreshold &&
					state.Reverse().Take(activeThreshold).All(S => S != HandState.Lasso && S != HandState.Unknown && S != HandState.NotTracked)
					||
					handBuffer.Reverse().Take(activeThreshold).All(J => J.TrackingState == TrackingState.NotTracked) &&
					state.Reverse().Take(activeThreshold).All(S => S == HandState.NotTracked))
				{
					isLasso = false;
				}
			}
			return isLasso;
		}
		private void IndicateState(bool active, Queue<Joint> handBuffer, Control panel, Color defaultActiveColor, Color defaultReadyColor)
		{
			if (active)
			{
				panel.BackColor = defaultActiveColor;
			}
			else
			{
				if (handBuffer.Count <= 0 || handBuffer.Last().TrackingState == TrackingState.NotTracked || handBuffer.Last().TrackingState == TrackingState.Inferred)
				{
					panel.BackColor = Color.Red;
				}
				else
				{
					if (IsCloserToHead(headBuffer.Last(), pelvisBuffer.Last(), handBuffer.Last()))
					{
						panel.BackColor = defaultReadyColor;
					}
					else
					{
						panel.BackColor = Color.Pink;
					}
				}
			}
		}

		private void MovingHand(Queue<Joint> handBuffer, Queue<TrackingConfidence> confidence, Queue<HandState> state, Control panel)
		{
			if (TestHandClosed(ref isMoving, handBuffer, confidence, state) && handBuffer.Count >= 2)
			{
				var CurrentPos = handBuffer.Last().Position;
				var PreviousPos = handBuffer.Reverse().Skip(1).First().Position;
				float X = CurrentPos.X - PreviousPos.X;
				float Y = CurrentPos.Y - PreviousPos.Y;
				var Resolution = Screen.FromControl(this).Bounds;
				float Width = Resolution.Width;
				float Height = Resolution.Height;
				mouse_move((int)(X * Width * sensitivityX), (int)(Y * -Height * sensitivityY));
			}
			IndicateState(isMoving, handBuffer, panel, Color.DarkGreen, Color.LightGreen);
		}
		private void ClickingHand(Queue<Joint> handBuffer, Queue<TrackingConfidence> confidence, Queue<HandState> state, Control panel)
		{
			TestHandClosed(ref isDown, handBuffer, confidence, state);
			TestHandLasso(ref isLasso, handBuffer, confidence, state);
			if (isDown)
			{
				mouse_down(true);
				IndicateState(isDown, handBuffer, panel, Color.DarkGreen, Color.LightGreen);
			}
			else if (isLasso)
			{
				mouse_down(false);
				IndicateState(isLasso, handBuffer, panel, Color.DarkBlue, Color.LightBlue);
			}
			else
			{
				mouse_up(true);
				mouse_up(false);
				if (state.LastOrDefault() == HandState.Lasso)
				{
					IndicateState(isLasso, handBuffer, panel, Color.DarkBlue, Color.LightBlue);
				}
				else IndicateState(isDown, handBuffer, panel, Color.DarkGreen, Color.LightGreen);
			}
		}

		private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
		{
			using (BodyFrame frame = e.FrameReference.AcquireFrame())
			{
				if (frame == null) return;
				if (bodies == null)
				{
					bodies = new Body[frame.BodyCount];
				}
				frame.GetAndRefreshBodyData(bodies);

				try
				{
					Body body = bodies.First((B) => B.IsTracked);
					headBuffer.Enqueue(body.Joints[JointType.Head]);
					pelvisBuffer.Enqueue(body.Joints[JointType.SpineBase]);
					leftHandBuffer.Enqueue(body.Joints[JointType.HandLeft]);
					rightHandBuffer.Enqueue(body.Joints[JointType.HandRight]);
					leftHandConfidence.Enqueue(body.HandLeftConfidence);
					leftHandState.Enqueue(body.HandLeftState);
					rightHandConfidence.Enqueue(body.HandRightConfidence);
					rightHandState.Enqueue(body.HandRightState);
					while (headBuffer.Count > bufferCount)
					{
						headBuffer.Dequeue();
					}
					while (pelvisBuffer.Count > bufferCount)
					{
						pelvisBuffer.Dequeue();
					}
					while (leftHandBuffer.Count > bufferCount)
					{
						leftHandBuffer.Dequeue();
					}
					while (rightHandBuffer.Count > bufferCount)
					{
						rightHandBuffer.Dequeue();
					}
					while (leftHandConfidence.Count > bufferCount)
					{
						leftHandConfidence.Dequeue();
					}
					while (leftHandState.Count > bufferCount)
					{
						leftHandState.Dequeue();
					}
					while (rightHandConfidence.Count > bufferCount)
					{
						rightHandConfidence.Dequeue();
					}
					while (rightHandState.Count > bufferCount)
					{
						rightHandState.Dequeue();
					}

					if (LeftMove)
					{
						MovingHand(leftHandBuffer, leftHandConfidence, leftHandState, panel1);
					}
					else
					{
						MovingHand(rightHandBuffer, rightHandConfidence, rightHandState, panel2);
					}

					if (LeftMove)
					{
						ClickingHand(rightHandBuffer, rightHandConfidence, rightHandState, panel2);
					}
					else
					{
						ClickingHand(leftHandBuffer, leftHandConfidence, leftHandState, panel1);
					}
				}
				catch
				{
					isMoving = false;
					isDown = false;
					mouse_up(true);
					mouse_up(false);
					panel1.BackColor = panel2.BackColor = BackColor;
				}
			}
		}

		bool leftDown = false, rightDown = false;
		private void mouse_down(bool left)
		{
			if (left && !leftDown)
			{
				mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, GetMessageExtraInfo());
				leftDown = true;
			}
			if (!left && !rightDown)
			{
				mouse_event(MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, GetMessageExtraInfo());
				rightDown = true;
			}
		}
		private void mouse_up(bool left)
		{
			if (left && leftDown)
			{
				mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, GetMessageExtraInfo());
				leftDown = false;
			}
			if (!left && rightDown)
			{
				mouse_event(MOUSEEVENTF_RIGHTUP, 0, 0, 0, GetMessageExtraInfo());
				rightDown = false;
			}
		}
		private void mouse_move(int X, int Y)
		{
			mouse_event(MOUSEEVENTF_MOVE, X, Y, 0, GetMessageExtraInfo());
		}

		internal const uint MOUSEEVENTF_MOVE = 0x0001;
		internal const uint MOUSEEVENTF_LEFTDOWN = 0x0002;
		internal const uint MOUSEEVENTF_LEFTUP = 0x0004;
		internal const uint MOUSEEVENTF_RIGHTDOWN = 0x0008;
		internal const uint MOUSEEVENTF_RIGHTUP = 0x0010;

		[DllImport("User32.dll")]
		internal static extern void mouse_event(uint dwflags, int X, int Y, uint dwData, IntPtr dwExtraInfo);

		[DllImport("User32.dll")]
		internal static extern IntPtr GetMessageExtraInfo();

		private void splitContainer1_SizeChanged(object sender, EventArgs e)
		{
			splitContainer1.SplitterDistance = splitContainer1.Width / 2 - (splitContainer1.SplitterWidth / 2);
		}

		bool LeftMove = true;
		private void button1_Click(object sender, EventArgs e)
		{
			LeftMove = !LeftMove;
			button1.Text = LeftMove ? "Left Hand Movement" : "Right Hand Movement";
		}
	}
}

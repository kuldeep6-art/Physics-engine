using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class Quaternion
	{
		// Real component
		public float R { get; set; }

		// First complex component
		public float I { get; set; }

		// Second complex component
		public float J { get; set; }

		// Third complex component
		public float K { get; set; }

		// Holds quaternion data in array form
		public float[] Data { get; set; }

		// Default constructor - initializes as identity quaternion (1, 0, 0, 0)
		public Quaternion()
		{
			R = 1f;
			I = 0f;
			J = 0f;
			K = 0f;
			Data = new float[] { 1f, 0f, 0f, 0f };
		}

		public Quaternion(float r, float i, float j, float k)
		{
			R = r;
			I = i;
			J = j;
			K = k;
			Data = new float[] { r, i, j, k };
		}

		public void Normalize()
		{
			float d = R * R + I * I + J * J + K * K;

			// Check for zero-length quaternion, and use the no-rotation quaternion in that case
			if (d == 0)
			{
				R = 1;
				return;
			}

			d = 1.0f / (float)Math.Sqrt(d);  // Normalize the quaternion

			R *= d;
			I *= d;
			J *= d;
			K *= d;
		}

		public void MultiplyAssign(Quaternion multiplier)
		{
			Quaternion q = this;  // Save the current quaternion
			R = q.R * multiplier.R - q.I * multiplier.I - q.J * multiplier.J - q.K * multiplier.K;
			I = q.R * multiplier.I + q.I * multiplier.R + q.J * multiplier.K - q.K * multiplier.J;
			J = q.R * multiplier.J + q.J * multiplier.R + q.K * multiplier.I - q.I * multiplier.K;
			K = q.R * multiplier.K + q.K * multiplier.R + q.I * multiplier.J - q.J * multiplier.I;
		}

		public void AddScaledVector(Vector3 vector, float scale)
		{
			Quaternion q = new Quaternion(0, vector.X * scale, vector.Y * scale, vector.Z * scale);
			q.MultiplyAssign(this); // Multiply the current quaternion by q
			R += q.R * 0.5f;
			I += q.I * 0.5f;
			J += q.J * 0.5f;
			K += q.K * 0.5f;
		}
	}

}

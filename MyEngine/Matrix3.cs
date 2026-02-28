using System;
using System.Collections.Generic;
using System.Text;

	namespace MyEngine
	{
		public class Matrix3
		{
			public float[] data = new float[9];

			public Matrix3(float[] values)
			{
				if (values.Length != 9)
					throw new ArgumentException("Matrix3 must have 9 elements.");
				data = values;
			}
		public Matrix3()
		{
			
		}
		public void SetComponents(Vector3 compOne, Vector3 compTwo, Vector3 compThree)
		{
			data[0] = compOne.X;  // Column 0
			data[1] = compOne.Y;
			data[2] = compOne.Z;
			data[3] = compTwo.X;  // Column 1
			data[4] = compTwo.Y;
			data[5] = compTwo.Z;
			data[6] = compThree.X; // Column 2
			data[7] = compThree.Y;
			data[8] = compThree.Z;
		}
		/// <summary>
		/// Multiplies this matrix by another Matrix3.
		/// </summary>
		public Matrix3 Multiply(Matrix3 o)
			{
				return new Matrix3(new float[]
				{
				data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
				data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
				data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],
				data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
				data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
				data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],
				data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
				data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
				data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
				});
			}
		public bool IsDegenerate()
		{
			float t4 = data[0] * data[4];
			float t6 = data[0] * data[5];
			float t8 = data[1] * data[3];
			float t10 = data[2] * data[3];
			float t12 = data[1] * data[6];
			float t14 = data[2] * data[6];

			float det = (t4 * data[8] - t6 * data[7] - t8 * data[8] +
						 t10 * data[7] + t12 * data[5] - t14 * data[4]);

			// Use a small epsilon to account for floating-point precision
			const float epsilon = 1e-6f;
			return Math.Abs(det) < epsilon;
		}
		public Matrix3 Inverse()
			{
				float t4 = data[0] * data[4];
				float t6 = data[0] * data[5];
				float t8 = data[1] * data[3];
				float t10 = data[2] * data[3];
				float t12 = data[1] * data[6];
				float t14 = data[2] * data[6];

				float det = (t4 * data[8] - t6 * data[7] - t8 * data[8] +
							 t10 * data[7] + t12 * data[5] - t14 * data[4]);

				if (det == 0.0f)
					throw new InvalidOperationException("Matrix is singular and cannot be inverted.");

				float invDet = 1.0f / det;

				return new Matrix3(new float[]
				{
				(data[4] * data[8] - data[5] * data[7]) * invDet,
				-(data[1] * data[8] - data[2] * data[7]) * invDet,
				(data[1] * data[5] - data[2] * data[4]) * invDet,
				-(data[3] * data[8] - data[5] * data[6]) * invDet,
				(data[0] * data[8] - t14) * invDet,
				-(t6 - t10) * invDet,
				(data[3] * data[7] - data[4] * data[6]) * invDet,
				-(data[0] * data[7] - t12) * invDet,
				(t4 - t8) * invDet
				});
			}

			public Vector3 Transform(Vector3 vector)
			{
				return new Vector3(
					this.data[0] * vector.X + this.data[1] * vector.Y + this.data[2] * vector.Z,
					this.data[3] * vector.X + this.data[4] * vector.Y + this.data[5] * vector.Z,
					this.data[6] * vector.X + this.data[7] * vector.Y + this.data[8] * vector.Z
				);
			}

			public Matrix3 Transpose()
			{
				return new Matrix3(new float[]
				{
				data[0], data[3], data[6],
				data[1], data[4], data[7],
				data[2], data[5], data[8]
				});
			}
		public Vector3 TransformTranspose(Vector3 vector)
		{
			// Assuming transpose transformation logic here
			return new Vector3(
				vector.X * data[0] + vector.Y * data[1] + vector.Z * data[2],
				vector.X * data[3] + vector.Y * data[4] + vector.Z * data[5],
				vector.X * data[6] + vector.Y * data[7] + vector.Z * data[8]);
		}
		public void SetSkewSymmetric(Vector3 vector)
		{
			data[0] = data[4] = data[8] = 0;
			data[1] = -vector.Z;
			data[2] = vector.Y;
			data[3] = vector.Z;
			data[5] = -vector.X;
			data[6] = -vector.Y;
			data[7] = vector.X;
		}
		public void SetOrientation(Quaternion q)
			{
				data[0] = 1 - (2 * q.J * q.J + 2 * q.K * q.K);
				data[1] = 2 * q.I * q.J + 2 * q.K * q.R;
				data[2] = 2 * q.I * q.K - 2 * q.J * q.R;
				data[3] = 2 * q.I * q.J - 2 * q.K * q.R;
				data[4] = 1 - (2 * q.I * q.I + 2 * q.K * q.K);
				data[5] = 2 * q.J * q.K + 2 * q.I * q.R;
				data[6] = 2 * q.I * q.K + 2 * q.J * q.R;
				data[7] = 2 * q.J * q.K - 2 * q.I * q.R;
				data[8] = 1 - (2 * q.I * q.I + 2 * q.J * q.J);
			}

			public static Matrix3 Lerp(Matrix3 a, Matrix3 b, float t)
			{
				t = Math.Max(0f, Math.Min(1f, t)); // Clamp t between 0 and 1
				float[] result = new float[9];
				for (int i = 0; i < 9; i++)
				{
					result[i] = a.data[i] + t * (b.data[i] - a.data[i]);
				}
				return new Matrix3(result);
			}
		public static Matrix3 operator *(Matrix3 a, Matrix3 b)
		{
			Matrix3 result = new Matrix3();
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					result.data[i * 3 + j] = a.data[i * 3 + 0] * b.data[0 * 3 + j] +
										   a.data[i * 3 + 1] * b.data[1 * 3 + j] +
										   a.data[i * 3 + 2] * b.data[2 * 3 + j];
			return result;
		}

		public static Matrix3 operator *(Matrix3 m, float scalar)
		{
			Matrix3 result = new Matrix3();
			for (int i = 0; i < 9; i++)
				result.data[i] = m.data[i] * scalar;
			return result;
		}

		public static Matrix3 operator +(Matrix3 a, Matrix3 b)
		{
			Matrix3 result = new Matrix3();
			for (int i = 0; i < 9; i++)
				result.data[i] = a.data[i] + b.data[i];
			return result;
		}
	}


	}

	

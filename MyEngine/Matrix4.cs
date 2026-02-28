using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class Matrix4
	{
		public  float[] data = new float[12];


		public Matrix4(float[] values)
		{
			if (values.Length != 16)
				throw new ArgumentException("Matrix4 must have 16 elements.");
			data = values;
		}

		/// <summary>
		/// Transforms the given vector by this matrix.
		/// </summary>
		/// <param name="vector">The vector to transform.</param>
		/// <returns>Transformed Vector3.</returns>
		public Vector3 Multiply(Vector3 vector)
		{
			return new Vector3(
				vector.X * data[0] + vector.Y * data[1] + vector.Z * data[2] + data[3],
				vector.X * data[4] + vector.Y * data[5] + vector.Z * data[6] + data[7],
				vector.X * data[8] + vector.Y * data[9] + vector.Z * data[10] + data[11]
			);
		}
		public Matrix4 Multiply(Matrix4 o)
		{
			float[] result = new float[16];

			for (int row = 0; row < 4; row++)
			{
				for (int col = 0; col < 4; col++)
				{
					result[row * 4 + col] =
						data[row * 4] * o.data[col] +
						data[row * 4 + 1] * o.data[col + 4] +
						data[row * 4 + 2] * o.data[col + 8] +
						data[row * 4 + 3] * o.data[col + 12];
				}
			}

			return new Matrix4(result);
		}
		public float GetDeterminant()
		{
			return data[8] * data[5] * data[2] +
				   data[4] * data[9] * data[2] +
				   data[8] * data[1] * data[6] -
				   data[0] * data[9] * data[6] -
				   data[4] * data[1] * data[10] +
				   data[0] * data[5] * data[10];
		}

		public Matrix4 Inverse()
		{
			float det = GetDeterminant();
			if (det == 0.0f)
				throw new InvalidOperationException("Matrix is singular and cannot be inverted.");

			float invDet = 1.0f / det;
			float[] invData = new float[16];

			invData[0] = (-data[9] * data[6] + data[5] * data[10]) * invDet;
			invData[4] = (data[8] * data[6] - data[4] * data[10]) * invDet;
			invData[8] = (-data[8] * data[5] + data[4] * data[9]) * invDet;
			invData[1] = (data[9] * data[2] - data[1] * data[10]) * invDet;
			invData[5] = (-data[8] * data[2] + data[0] * data[10]) * invDet;
			invData[9] = (data[8] * data[1] - data[0] * data[9]) * invDet;
			invData[2] = (-data[5] * data[2] + data[1] * data[6]) * invDet;
			invData[6] = (data[4] * data[2] - data[0] * data[6]) * invDet;
			invData[10] = (-data[4] * data[1] + data[0] * data[5]) * invDet;

			return new Matrix4(invData);
		}
		


		public void SetOrientationAndPos(Quaternion q, Vector3 pos)
		{
			data[0] = 1 - (2 * q.J * q.J + 2 * q.K * q.K);
			data[1] = 2 * q.I * q.J + 2 * q.K * q.R;
			data[2] = 2 * q.I * q.K - 2 * q.J * q.R;
			data[3] = pos.X;

			data[4] = 2 * q.I * q.J - 2 * q.K * q.R;
			data[5] = 1 - (2 * q.I * q.I + 2 * q.K * q.K);
			data[6] = 2 * q.J * q.K + 2 * q.I * q.R;
			data[7] = pos.Y;

			data[8] = 2 * q.I * q.K + 2 * q.J * q.R;
			data[9] = 2 * q.J * q.K - 2 * q.I * q.R;
			data[10] = 1 - (2 * q.I * q.I + 2 * q.J * q.J);
			data[11] = pos.Z;
		}
		public Vector3 TransformInverse(Vector3 vector)
		{
			// Apply the inverse of the position transformation
			Vector3 tmp = new Vector3(
				vector.X - data[3],
				vector.Y - data[7],
				vector.Z - data[11]
			);

			// Perform the matrix-vector multiplication
			return new Vector3(
				tmp.X * data[0] + tmp.Y * data[4] + tmp.Z * data[8],
				tmp.X * data[1] + tmp.Y * data[5] + tmp.Z * data[9],
				tmp.X * data[2] + tmp.Y * data[6] + tmp.Z * data[10]
			);
		}
		public Vector3 TransformDirection(Vector3 vector)
		{
			return new Vector3(
				vector.X * data[0] + vector.Y * data[1] + vector.Z * data[2],
				vector.X * data[4] + vector.Y * data[5] + vector.Z * data[6],
				vector.X * data[8] + vector.Y * data[9] + vector.Z * data[10]
			);
		}

		/// <summary>
		/// Transforms the given direction vector by the inverse of this matrix.
		/// </summary>
		public Vector3 TransformInverseDirection(Vector3 vector)
		{
			return new Vector3(
				vector.X * data[0] + vector.Y * data[4] + vector.Z * data[8],
				vector.X * data[1] + vector.Y * data[5] + vector.Z * data[9],
				vector.X * data[2] + vector.Y * data[6] + vector.Z * data[10]
			);
		}
	}

}

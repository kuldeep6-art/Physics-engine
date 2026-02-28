using System;
using System.Numerics;
using MyEngine.warppers;

namespace MyEngine
{
	public class Vector3
	{
		public float X { get; set; }
		public float Y { get; set; }
		public float Z { get; set; }

		private float pad; // Unused, consider removing unless needed

		public Vector3()
		{
			X = 0f;
			Y = 0f;
			Z = 0f;
		}

		public Vector3(float x, float y, float z)
		{
			X = x;
			Y = y;
			Z = z;
		}

		public void Invert()
		{
			X = -X;
			Y = -Y;
			Z = -Z;
		}

		public float Magnitude()
		{
			return MathF.Sqrt(X * X + Y * Y + Z * Z);
		}

		public float SquareMagnitude()
		{
			return X * X + Y * Y + Z * Z;
		}

		public void normalize()
		{
			float length = Magnitude();
			if (length > 0)
			{
				X /= length;
				Y /= length;
				Z /= length;
			}
		}

		public static Vector3 operator *(Vector3 v, float scalar)
		{
			return new Vector3(v.X * scalar, v.Y * scalar, v.Z * scalar);
		}

		public static Vector3 operator +(Vector3 v1, Vector3 v2)
		{
			return new Vector3(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
		}

		public static Vector3 operator -(Vector3 v1, Vector3 v2)
		{
			return new Vector3(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
		}

		public Vector3 addScaledVector(Vector3 v1, Vector3 v2, float scale)
		{
			return new Vector3(v1.X + v2.X * scale, v1.Y + v2.Y * scale, v1.Z + v2.Z * scale);
		}

		public Vector3 ComponentProduct(Vector3 v1, Vector3 v2)
		{
			return new Vector3(v1.X * v2.X, v1.Y * v2.Y, v1.Z * v2.Z);
		}

		public void ComponentProductUpdate(Vector3 v1, Vector3 v2)
		{
			v1.X *= v2.X;
			v1.Y *= v2.Y;
			v1.Z *= v2.Z;
		}

		public float ScalarProduct(Vector3 v1, Vector3 v2)
		{
			return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
		}

		public static float operator *(Vector3 v1, Vector3 v2)
		{
			return v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z;
		}

		public Vector3 vectorProduct(Vector3 vector)
		{
			return new Vector3(
				Y * vector.Z - Z * vector.Y,
				Z * vector.X - X * vector.Z,
				X * vector.Y - Y * vector.X
			);
		}

		public void VectorProductAssign(Vector3 vector)
		{
			Vector3 result = vectorProduct(vector);
			X = result.X;
			Y = result.Y;
			Z = result.Z;
		}

		public static Vector3 operator ^(Vector3 v1, Vector3 v2)
		{
			return new Vector3(
				v1.Y * v2.Z - v1.Z * v2.Y,
				v1.Z * v2.X - v1.X * v2.Z,
				v1.X * v2.Y - v1.Y * v2.X
			);
		}

		public static void MakeOrthonormalBasis(Vector3 x, ref Vector3 y, out Vector3 z)
		{
			z = x.vectorProduct(y);
			float squaredMag = z.X * z.X + z.Y * z.Y + z.Z * z.Z;
			if (squaredMag == 0.0f)
			{
				z = new Vector3(0, 0, 0);
				return;
			}

			float mag = (float)Math.Sqrt(squaredMag);
			z = new Vector3(z.X / mag, z.Y / mag, z.Z / mag);
			y = z.vectorProduct(x);
		}
		public float CalculateDistance(Vector3 vector1, Vector3 vector2)
		{
			float dx = vector2.X - vector1.X;
			float dy = vector2.Y - vector1.Y;
			float dz = vector2.Z - vector1.Z;
			return (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
		}
	}
}

using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class Primitive
	{
		public RigidBody Body { get; set; }
		public Matrix4 Offset { get; set; }

		// Constructor for initialization
		public Primitive(RigidBody body, Matrix4 offset)
		{
			Body = body;
			Offset = offset;
		}
	}
	public class Sphere : Primitive
	{
		public float Radius { get; set; }

		// Constructor
		public Sphere(RigidBody body, Matrix4 offset, float radius)
			: base(body, offset)
		{
			Radius = radius;
		}
	}
	public class Plane : Primitive
	{
		public Vector3 Normal { get; set; }
		public float Offset { get; set; }

		// Constructor
		public Plane(RigidBody body, Matrix4 offset, Vector3 normal, float offsetValue)
			: base(body, offset)
		{
			Normal = normal;
			Offset = offsetValue;
		}
	}
	public class Box : Primitive
	{
		public Vector3 HalfSize { get; set; }

		public Box(RigidBody body, Matrix4 offset, Vector3 halfSize)
			: base(body, offset)
		{
			HalfSize = halfSize;
		}

		// Helper method to generate all vertices in world coordinates
		public Vector3[] GetVertices()
		{
			Vector3[] vertices = new Vector3[8]
			{
			new Vector3(-HalfSize.X, -HalfSize.Y, -HalfSize.Z),
			new Vector3(-HalfSize.X, -HalfSize.Y, +HalfSize.Z),
			new Vector3(-HalfSize.X, +HalfSize.Y, -HalfSize.Z),
			new Vector3(-HalfSize.X, +HalfSize.Y, +HalfSize.Z),
			new Vector3(+HalfSize.X, -HalfSize.Y, -HalfSize.Z),
			new Vector3(+HalfSize.X, -HalfSize.Y, +HalfSize.Z),
			new Vector3(+HalfSize.X, +HalfSize.Y, -HalfSize.Z),
			new Vector3(+HalfSize.X, +HalfSize.Y, +HalfSize.Z)
			};

			// Transform vertices to world coordinates using Offset
			for (int i = 0; i < 8; i++)
			{
				vertices[i] = Offset.Multiply(vertices[i]);
			}

			return vertices;
		}
	}
}

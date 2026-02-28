using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{

	public struct BoundingBox
	{
		public Vector3 Center;
		public Vector3 HalfSize;

		public BoundingBox(Vector3 center, Vector3 halfSize)
		{
			Center = center;
			HalfSize = halfSize;
		}
	}
}

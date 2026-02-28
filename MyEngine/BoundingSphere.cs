using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public struct BoundingSphere : IBoundingVolume
	{
		public Vector3 Center { get; set; }
		public float Radius { get; set; }

		public BoundingSphere(Vector3 center, float radius)
		{
			Center = center;
			Radius = radius;
		}

		public BoundingSphere(BoundingSphere one, BoundingSphere two)
		{
			Vector3 centerOffset = two.Center - one.Center;
			float distance = centerOffset.SquareMagnitude();
			float radiusDiff = two.Radius - one.Radius;

			if (radiusDiff * radiusDiff >= distance)
			{
				if (one.Radius > two.Radius)
				{
					Center = one.Center;
					Radius = one.Radius;
				}
				else
				{
					Center = two.Center;
					Radius = two.Radius;
				}
			}
			else
			{
				distance = (float)Math.Sqrt(distance);
				Radius = (distance + one.Radius + two.Radius) * 0.5f;
				Center = one.Center;
				if (distance > 0)
				{
					Center += centerOffset * ((Radius - one.Radius) / distance);
				}
			}
		}

		public bool Overlaps(IBoundingVolume other)
		{
			if (other is BoundingSphere otherSphere)
			{
				float distanceSquared = (Center - otherSphere.Center).SquareMagnitude();
				return distanceSquared < (Radius + otherSphere.Radius) * (Radius + otherSphere.Radius);
			}
			return false;
		}

		public float GetSize()
		{
			return Radius;
		}
		public float GetGrowth(IBoundingVolume other)
		{
			if (other is BoundingSphere sphere)
			{
				BoundingSphere newSphere = new BoundingSphere(this, sphere);
				return (newSphere.Radius * newSphere.Radius) - (this.Radius * this.Radius);
			}

			throw new InvalidOperationException("GetGrowth only supports BoundingSphere type currently.");
		}
		public IBoundingVolume Combine(IBoundingVolume other)
		{
			if (other is BoundingSphere sphere)
			{
				return new BoundingSphere(this, sphere);
			}

			throw new InvalidOperationException("Combine only supports BoundingSphere.");
		}

	}
}

using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public interface IBoundingVolume
	{
		bool Overlaps(IBoundingVolume other);
		float GetSize();
		float GetGrowth(IBoundingVolume other);
		IBoundingVolume Combine(IBoundingVolume other);
	}
}

using System;
using System.Collections.Generic;
using System.Text;
using MyEngine.warppers;

namespace MyEngine
{
	public struct Grid
	{
		public uint xExtent;        // Number of cells in x direction
		public uint zExtent;        // Number of cells in z direction
		public ObjectSet<uint>[] locations;  // Array of size (xExtent * zExtent)
		public Vector3 origin;      // Origin point of the grid
		public Vector3 oneOverCellSize;  // 1 divided by cell size for each dimension

		// Constructor to initialize the grid
		public Grid(uint xExtent, uint zExtent, Vector3 origin, Vector3 cellSize)
		{
			this.xExtent = xExtent;
			this.zExtent = zExtent;
			this.locations = new ObjectSet<uint>[xExtent * zExtent];
			this.origin = origin;
			this.oneOverCellSize = new Vector3(
				1f / cellSize.X,
				1f,  // y component typically set to 1
				1f / cellSize.Z
			);
		}

		// Method to get the index of the location containing an object's center
		public uint GetLocationIndex(Vector3 objectPosition)
		{
			// Calculate position relative to origin
			Vector3 relativePos = objectPosition - origin;

			// Component-wise multiplication with oneOverCellSize
			Vector3 square = new Vector3(
				relativePos.X * oneOverCellSize.X,
				relativePos.Y * oneOverCellSize.Y,
				relativePos.Z * oneOverCellSize.Z
			);

			// Convert to grid coordinates and calculate index
			return (uint)square.X + xExtent * (uint)square.Z;
		}
	}
}

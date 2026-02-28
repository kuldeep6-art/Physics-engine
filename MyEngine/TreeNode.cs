using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{

	public struct QuadTreeNode
	{
		public Vector3 position;
		public QuadTreeNode[] child;

		public QuadTreeNode(Vector3 pos)
		{
			position = pos;
			child = new QuadTreeNode[4];
		}

		public uint GetChildIndex(Vector3 objectPos)
		{
			uint index = 0;
			if (objectPos.X > position.X) index += 1;
			if (objectPos.Z > position.Z) index += 2;
			return index;
		}
	}

	public struct OctTreeNode
	{
		public Vector3 position;
		public OctTreeNode[] child;

		public OctTreeNode(Vector3 pos)
		{
			position = pos;
			child = new OctTreeNode[8];
		}

		public uint GetChildIndex(Vector3 objectPos)
		{
			uint index = 0;
			if (objectPos.X > position.X) index += 1;
			if (objectPos.Y > position.Y) index += 2;
			if (objectPos.Z > position.Z) index += 4;
			return index;
		}
	}
}

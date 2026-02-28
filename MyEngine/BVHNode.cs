using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MyEngine
{
	public class BVHNode<T> where T : IBoundingVolume
	{
		public BVHNode<T>[] Children { get; private set; }
		public T Volume { get; set; }
		public RigidBody Body { get; set; }
		public BVHNode<T> Parent { get; private set; }

		public BVHNode(BVHNode<T> parent = null, T volume = default, RigidBody body = null)
		{
			Children = new BVHNode<T>[2];
			Parent = parent;
			Volume = volume;
			Body = body;
		}

		public bool IsLeaf()
		{
			return Body != null;
		}

		public uint GetPotentialContacts(PotentialContact[] contacts, uint limit)
		{
			if (IsLeaf() || limit == 0) return 0;
			return Children[0].GetPotentialContactsWith(Children[1], contacts, limit);
		}

		public uint GetPotentialContactsWith(BVHNode<T> other, PotentialContact[] contacts, uint limit)
		{
			if (!Overlaps(other) || limit == 0) return 0;

			if (IsLeaf() && other.IsLeaf())
			{
				contacts[0] = new PotentialContact(Body, other.Body);
				return 1;
			}

			if (other.IsLeaf() || (!IsLeaf() && Volume.GetSize() >= other.Volume.GetSize()))
			{
				uint count = Children[0].GetPotentialContactsWith(other, contacts, limit);
				if (limit > count)
				{
					return count + Children[1].GetPotentialContactsWith(other, contacts.Skip((int)count).ToArray(), limit - count);
				}
				return count;
			}
			else
			{
				uint count = GetPotentialContactsWith(other.Children[0], contacts, limit);
				if (limit > count)
				{
					return count + GetPotentialContactsWith(other.Children[1], contacts.Skip((int)count).ToArray(), limit - count);
				}
				return count;
			}
		}

		public bool Overlaps(BVHNode<T> other)
		{
			return Volume.Overlaps(other.Volume);
		}

		public void Insert(RigidBody newBody, T newVolume)
		{
			if (IsLeaf())
			{
				Children[0] = new BVHNode<T>(this, Volume, Body);
				Children[1] = new BVHNode<T>(this, newVolume, newBody);
				Body = null;
				RecalculateBoundingVolume();
			}
			else
			{
				if (Children[0].Volume.GetGrowth(newVolume) < Children[1].Volume.GetGrowth(newVolume))
				{
					Children[0].Insert(newBody, newVolume);
				}
				else
				{
					Children[1].Insert(newBody, newVolume);
				}
			}
		}

		private void RecalculateBoundingVolume()
		{
			if (Children[0] != null && Children[1] != null)
			{
				Volume = (T)Children[0].Volume.Combine(Children[1].Volume);
			}
		}

		public void Destroy()
		{
			if (Parent != null)
			{
				BVHNode<T> sibling = Parent.Children[0] == this ? Parent.Children[1] : Parent.Children[0];
				Parent.Volume = sibling.Volume;
				Parent.Body = sibling.Body;
				Parent.Children[0] = sibling.Children[0];
				Parent.Children[1] = sibling.Children[1];
				if (Parent.Children[0] != null) Parent.Children[0].Parent = Parent;
				if (Parent.Children[1] != null) Parent.Children[1].Parent = Parent;
				sibling.Parent = null;
				sibling.Body = null;
				sibling.Children[0] = null;
				sibling.Children[1] = null;
				Parent.RecalculateBoundingVolume();
			}
			if (Children[0] != null)
			{
				Children[0].Parent = null;
				Children[0] = null;
			}
			if (Children[1] != null)
			{
				Children[1].Parent = null;
				Children[1] = null;
			}
		}
	}

}

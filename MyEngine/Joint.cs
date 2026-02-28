using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Text;

namespace MyEngine
{
	public class Joint
	{
		/// <summary>
		/// Holds the two rigid bodies that are connected by this joint.
		/// </summary>
		public RigidBody[] Body { get; set; } = new RigidBody[2];

		/// <summary>
		/// Holds the relative location of the connection for each
		/// body, given in local coordinates.
		/// </summary>
		public Vector3[] Position { get; set; } = new Vector3[2];

		/// <summary>
		/// Holds the maximum displacement at the joint before the
		/// joint is considered to be violated. This is normally a
		/// small, epsilon value. It can be larger, however, in which
		/// case the joint will behave as if an inelastic cable joined
		/// the bodies at their joint locations.
		/// </summary>
		public float Error { get; set; }

		/// <summary>
		/// Generates the contacts required to restore the joint if it
		/// has been violated.
		/// </summary>
		/// <param name="contact">The contact object to populate</param>
		/// <param name="limit">Maximum number of contacts to generate</param>
		/// <returns>The number of contacts generated</returns>
		public uint AddContact(ParticleContact contact, uint limit)
		{
			// Implementation would go here
			throw new NotImplementedException();
		}
	}
}

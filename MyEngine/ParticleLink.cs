using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public abstract class ParticleLink
	{
		/// <summary>
		/// Holds the pair of particles that are connected by this link.
		/// </summary>
		public Particle[] particles = new Particle[2];

		/// <summary>
		/// Returns the current length of the link.
		/// </summary>
		protected float CurrentLength()
		{
			Vector3 relativePos = particles[0].Position - particles[1].Position;
			return relativePos.Magnitude();
		}

		/// <summary>
		/// Fills the given contact structure with the contact needed
		/// to keep the link from violating its constraint.
		/// </summary>
		public abstract uint FillContact(ParticleContact contact, uint limit);
	}

}

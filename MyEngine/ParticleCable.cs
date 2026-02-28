using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	
	/// <summary>
	/// Cables link a pair of particles, generating a contact if they
	/// stray too far apart.
	/// </summary>
	public class ParticleCable : ParticleLink
	{
		/// <summary>
		/// Holds the maximum length of the cable.
		/// </summary>
		public float MaxLength;

		/// <summary>
		/// Holds the restitution (bounciness) of the cable.
		/// </summary>
		public float Restitution;

		/// <summary>
		/// Fills the given contact structure with the contact needed
		/// to keep the cable from overextending.
		/// </summary>
		public override uint FillContact(ParticleContact contact, uint limit)
		{
			// Find the length of the cable.
			float length = CurrentLength();

			// Check whether we’re overextended.
			if (length < MaxLength)
			{
				return 0;
			}

			// Otherwise, return the contact.
			contact.Particles[0] = particles[0];
			contact.Particles[1] = particles[1];

			// Calculate the normal.
			Vector3 normal = particles[1].Position - particles[0].Position;
			normal.normalize();
			contact.ContactNormal = normal;
			contact.penetration = length - MaxLength;
			contact.Restitution = Restitution;
			return 1;
		}
	}

}

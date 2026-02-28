using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class ParticleRod : ParticleLink
	{
		/// <summary>
		/// Holds the length of the rod.
		/// </summary>
		public float Length;

		/// <summary>
		/// Fills the given contact structure with the contact needed
		/// to keep the rod from extending or compressing.
		/// </summary>
		public override uint FillContact(ParticleContact contact, uint limit)
		{
			// Find the length of the rod.
			float currentLen = CurrentLength();

			// Check whether we’re overextended.
			if (currentLen == Length)
			{
				return 0;
			}

			// Otherwise, return the contact.
			contact.Particles[0] = particles[0];
			contact.Particles[1] = particles[1];

			// Calculate the normal.
			Vector3 normal = particles[1].Position - particles[0].Position;
			normal.normalize();

			// The contact normal depends on whether we’re extending or compressing.
			if (currentLen > Length)
			{
				contact.ContactNormal = normal;
				contact.penetration = currentLen - Length;
			}
			else
			{
				contact.ContactNormal = normal * -1;
				contact.penetration = Length - currentLen;
			}

			// Always use zero restitution (no bounciness).
			contact.Restitution = 0;
			return 1;
		}
	}
}

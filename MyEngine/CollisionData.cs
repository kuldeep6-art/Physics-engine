using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Text;

namespace MyEngine
{
	public struct CollisionData
	{
		public ParticleContact[] Contacts;
		public uint ContactsLeft;
		public float Restitution; // Added for this context
		public float Friction;    // Added for this context
		public float Tolerance; // Added for vertex-plane check

		public CollisionData(ParticleContact[] contacts, uint contactsLeft, float restitution = 0f, float friction = 0f, float tolerance = 0f)
		{
			Contacts = contacts;
			ContactsLeft = contactsLeft;
			Restitution = restitution;
			Friction = friction;
			Tolerance = tolerance;
		}
	}
}

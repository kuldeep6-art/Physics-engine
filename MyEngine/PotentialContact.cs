using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{

	public struct PotentialContact
	{
		public RigidBody[] Body { get; set; }

		public PotentialContact(RigidBody body1, RigidBody body2)
		{
			Body = new RigidBody[2];
			Body[0] = body1;
			Body[1] = body2;
		}
		

	}
}

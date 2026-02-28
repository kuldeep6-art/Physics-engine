using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
    interface IBodyForceGenerator
    {
		void UpdateForce(RigidBody body, float duration);
	}
}

using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public interface IParticleContactGenerator
	{
		int AddContact(ParticleContact[] contacts, int limit);
	}
}

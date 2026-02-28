using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class ParticleWorld
	{
		private class ParticleRegistration
		{
			public Particle Particle;
			public ParticleRegistration Next;
		}
		private class BodyRegistration
		{
			public RigidBody body;
			public BodyRegistration Next;
		}

		private BodyRegistration firstBody;
		private class ContactGenRegistration
		{
			public IParticleContactGenerator Gen;
			public ContactGenRegistration Next;
		}

		private ParticleRegistration firstParticle;
		private ContactGenRegistration firstContactGen;
		private ParticleForceRegistry registry = new ParticleForceRegistry();
		private ParticleContactResolver resolver = new ParticleContactResolver();
		private ParticleContact[] contacts;
		private uint maxContacts;
		private bool calculateIterations = true;

		public ParticleWorld(uint maxContacts, uint iterations = 0)
		{
			this.maxContacts = maxContacts;
			contacts = new ParticleContact[maxContacts];
		}

		public void StartFrame()
		{
			ParticleRegistration reg = firstParticle;
			while (reg != null)
			{
				reg.Particle.ClearAccumulator();
				reg = reg.Next;
			}
		}
		public void StartFrameB()
		{
			BodyRegistration reg = firstBody;
			while (reg != null)
			{
				reg.body.ClearAccumulators();
				reg.body.CalculateDerivedData();
				reg = reg.Next;
			}
		}

		public uint GenerateContacts()
		{
			uint limit = maxContacts;
			uint usedContacts = 0;
			ParticleContact[] nextContact = contacts;
			ContactGenRegistration reg = firstContactGen;
			while (reg != null)
			{
				int used = reg.Gen.AddContact(nextContact, (int)limit);
				limit -= (uint)used;
				usedContacts += (uint)used;
				if (limit <= 0) break;
				reg = reg.Next;
			}
			return usedContacts;
		}

		public void Integrate(float duration)
		{
			ParticleRegistration reg = firstParticle;
			while (reg != null)
			{
				reg.Particle.Integrate(duration);
				reg = reg.Next;
			}
		}

		public void RunPhysics(float duration)
		{
			registry.UpdateForces(duration);
			Integrate(duration);
			uint usedContacts = GenerateContacts();
			if (calculateIterations)
				resolver.SetIterations((uint)usedContacts * 2);
			resolver.ResolveContacts(contacts, (uint)usedContacts, duration);
		}
	
	   public void RunPhysicsB(float duration)
		{
			// First apply the force generators
			// registry.UpdateForces(duration);

			// Then integrate the objects
			BodyRegistration reg = firstBody;
			while (reg != null)
			{
				reg.body.Integrate(duration);
				reg = reg.Next;
			}
		}
		public void AddBody(RigidBody body)
		{
			var newReg = new BodyRegistration
			{
				body = body,
				Next = firstBody
			};
			firstBody = newReg;
		}

		public void RemoveBody(RigidBody body)
		{
			BodyRegistration previous = null;
			BodyRegistration current = firstBody;

			while (current != null)
			{
				if (current.body == body)
				{
					if (previous != null)
					{
						previous.Next = current.Next;
					}
					else
					{
						firstBody = current.Next;
					}
					return;
				}
				previous = current;
				current = current.Next;
			}
		}

		public IEnumerable<RigidBody> GetBodies()
		{
			BodyRegistration current = firstBody;
			while (current != null)
			{
				yield return current.body;
				current = current.Next;
			}
		}

		public List<ParticleContact> CurrentFrameContacts { get; } = new List<ParticleContact>();

		public void ClearContacts()
		{
			CurrentFrameContacts.Clear();
		}

		public void AddContact(ParticleContact contact)
		{
			CurrentFrameContacts.Add(contact);
		}
	}
}


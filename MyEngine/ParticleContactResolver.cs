using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Text;

namespace MyEngine
{
	public class ParticleContactResolver
	{
		/// <summary>
		/// Holds the number of iterations allowed.
		/// </summary>
		protected uint iterations;
		protected uint VelocityIterations { get; set; } = 10;

		/// <summary>
		/// This is a performance tracking value - we keep a record
		/// of the actual number of iterations used.
		/// </summary>
		protected uint iterationsUsed;
		protected float PositionEpsilon { get; set; } = 0.01f;
		/// <summary>
		/// Creates a new contact resolver.
		/// </summary>
		/// 
		/// 
		public ParticleContactResolver()
		{
			this.iterations = 0;
		}
		public ParticleContactResolver(uint iterations)
		{
			this.iterations = iterations;
		}

		/// <summary>
		/// Sets the number of iterations that can be used.
		/// </summary>
		public void SetIterations(uint iterations)
		{
			this.iterations = iterations;
		}
		public void ResolveContacts(ParticleContact[] contactArray, uint numContacts, float duration)
		{
			// Make sure we have something to do
			if (numContacts == 0) return;

			// Prepare the contacts for processing
			PrepareContacts(contactArray, numContacts, duration);

			// Resolve the interpenetration problems with the contacts
			AdjustPositions(contactArray, numContacts, duration);

			// Resolve the velocity problems with the contacts
			AdjustVelocities(contactArray, numContacts, duration);
		}

		protected void PrepareContacts(ParticleContact[] contacts, uint numContacts, float duration)
		{
			foreach (ParticleContact contact in contacts)
			{
				if (contact == null) break;
				contact.CalculateInternals(duration);
			}
		}

		public void AdjustPositions(ParticleContact[] contacts, uint numContacts, float duration)
		{
			iterationsUsed = 0;
			while (iterationsUsed < iterations)
			{
				float max = PositionEpsilon;
				int index = (int)numContacts;
				for (int i = 0; i < numContacts; i++)
				{
					if (contacts[i].penetration > max)
					{
						max = contacts[i].penetration;
						index = i;
					}
				}
				if (index == numContacts) break;

				contacts[index].ApplyPositionChange(out Vector3[] velocityChange, out Vector3[] rotationChange, out float[] rotationAmount, max);

				for (int i = 0; i < numContacts; i++)
				{
					if (contacts[i].Body[0] != null)
					{
						if (contacts[i].Body[0] == contacts[index].Body[0])
							UpdatePenetration(contacts[i], 0, 0, velocityChange, rotationChange, rotationAmount);
						else if (contacts[i].Body[0] == contacts[index].Body[1])
							UpdatePenetration(contacts[i], 0, 1, velocityChange, rotationChange, rotationAmount);
					}
					if (contacts[i].Body[1] != null)
					{
						if (contacts[i].Body[1] == contacts[index].Body[0])
							UpdatePenetration(contacts[i], 1, 0, velocityChange, rotationChange, rotationAmount);
						else if (contacts[i].Body[1] == contacts[index].Body[1])
							UpdatePenetration(contacts[i], 1, 1, velocityChange, rotationChange, rotationAmount);
					}
				}
				iterationsUsed++;
			}
		}
		private void UpdatePenetration(ParticleContact contact, int bodyIndex, int changedBodyIndex, Vector3[] velocityChange, Vector3[] rotationChange, float[] rotationAmount)
		{
			Vector3 cp = rotationChange[changedBodyIndex].vectorProduct(contact.RelativeContactPosition[bodyIndex]) + velocityChange[changedBodyIndex];
			contact.penetration += (bodyIndex == changedBodyIndex ? -1 : 1) * rotationAmount[changedBodyIndex] * cp.ScalarProduct(cp,contact.ContactNormal);
		}
		// Alternative sorted list approach (simplified)
		public void AdjustPositionsSorted(ParticleContact orderedList, uint positionIterations)
		{
			ParticleContact adjustedList = null;
			for (uint i = 0; i < positionIterations; i++)
			{
				if (orderedList?.penetration < 0) break;

				orderedList.ApplyPositionChange(out Vector3[] velocityChange, out Vector3[] rotationChange, out float[] rotationAmount, orderedList.penetration);

				ParticleContact bodyContact = orderedList.Body[0]?.Contacts;
				while (bodyContact != null)
				{
					bodyContact.UpdatePenetration(velocityChange, rotationChange, rotationAmount);
					MoveToAdjusted(ref adjustedList, ref bodyContact);
					bodyContact = bodyContact.NextObject[bodyContact.Body[0] == orderedList.Body[0] ? 0 : 1];
				}

				if (orderedList.Body[1] != null)
				{
					bodyContact = orderedList.Body[1].Contacts;
					while (bodyContact != null)
					{
						bodyContact.UpdatePenetration(velocityChange, rotationChange, rotationAmount);
						MoveToAdjusted(ref adjustedList, ref bodyContact);
						bodyContact = bodyContact.NextObject[bodyContact.Body[1] == orderedList.Body[1] ? 1 : 0];
					}
				}

				SortInPlace(ref adjustedList);
				MergeSortedLists(ref orderedList, ref adjustedList);
			}
		}

		private void MoveToAdjusted(ref ParticleContact adjustedList, ref ParticleContact contact)
		{
			// Simplified list manipulation
			contact.PreviousInOrder.NextInOrder = contact.NextInOrder;
			contact.NextInOrder.PreviousInOrder = contact.PreviousInOrder;
			contact.NextInOrder = adjustedList;
			if (adjustedList != null) adjustedList.PreviousInOrder = contact;
			adjustedList = contact;
			contact.PreviousInOrder = null;
		}

		private void SortInPlace(ref ParticleContact list)
		{
			// Simplified bubble sort for demonstration
			ParticleContact current = list;
			while (current != null)
			{
				ParticleContact next = current.NextInOrder;
				while (next != null)
				{
					if (current.penetration < next.penetration)
					{
						SwapContacts(ref current, ref next);
					}
					next = next.NextInOrder;
				}
				current = current.NextInOrder;
			}
		}

		private void SwapContacts(ref ParticleContact a, ref ParticleContact b)
		{
			float tempPenetration = a.penetration;
			a.penetration = b.penetration;
			b.penetration = tempPenetration;
			// Swap other properties as needed
		}

		private void MergeSortedLists(ref ParticleContact orderedList, ref ParticleContact adjustedList)
		{
			ParticleContact dummy = new ParticleContact();
			ParticleContact tail = dummy;

			while (orderedList != null && adjustedList != null)
			{
				if (adjustedList.penetration > orderedList.penetration)
				{
					tail.NextInOrder = adjustedList;
					adjustedList.PreviousInOrder = tail;
					adjustedList = adjustedList.NextInOrder;
				}
				else
				{
					tail.NextInOrder = orderedList;
					orderedList.PreviousInOrder = tail;
					orderedList = orderedList.NextInOrder;
				}
				tail = tail.NextInOrder;
			}

			tail.NextInOrder = orderedList ?? adjustedList;
			if (tail.NextInOrder != null) tail.NextInOrder.PreviousInOrder = tail;
			orderedList = dummy.NextInOrder;
			if (orderedList != null) orderedList.PreviousInOrder = null;
		}
		public void AdjustVelocities(ParticleContact[] contacts, uint numContacts, float duration)
		{
			uint iterationsUsed = 0;
			while (iterationsUsed < VelocityIterations)
			{
				float maxClosingVelocity = 0;
				int index = (int)numContacts;
				for (int i = 0; i < numContacts; i++)
				{
					contacts[i].CalculateDesiredDeltaVelocity(duration);
					if (contacts[i].contactVelocity.X < 0 && -contacts[i].contactVelocity.X > maxClosingVelocity)
					{
						maxClosingVelocity = -contacts[i].contactVelocity.X;
						index = i;
					}
				}
				if (index == numContacts) break;

				// Apply velocity change (simplified - actual impulse application would be here)
				// Update other contacts based on changes (not fully implemented here)
				iterationsUsed++;
			}
		}

		/// <summary>
		/// Resolves a set of particle contacts for both penetration and velocity.
		/// </summary>
		/// 
		public void ResolveContactsP(ParticleContact[] contactArray, uint numContacts, float duration)
		{
			iterationsUsed = 0;
			while (iterationsUsed < iterations)
			{
				// Find the contact with the largest closing velocity
				float max = 0;
				int maxIndex = -1;
				for (int i = 0; i < numContacts; i++)
				{
					float sepVel = contactArray[i].CalculateSeparatingVelocity();
					if (sepVel < max)
					{
						max = sepVel;
						maxIndex = i;
					}
				}

				// If no valid contact is found, break out of loop
				if (maxIndex == -1)
					break;

				// Resolve this contact
				contactArray[maxIndex].Resolve(duration);
				iterationsUsed++;
			}
		}
	}
}

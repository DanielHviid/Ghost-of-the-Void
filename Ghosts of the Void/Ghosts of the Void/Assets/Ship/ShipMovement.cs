using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class ShipMovement : Ship
{

	// Public information
	public Vector3 targetLocation;

	// Target information
	Vector3 targetDirection;
	float targetDistance;
	Vector3 targetCrossProduct;
	Vector2 targetAngles;

	// Ship information
	Vector3 velocity;
	Vector3 bearing;
	Vector3 adjustedBearings;       // adjusts to reduce sideways circular movement
	Vector3 adjustedCross;          // adjusts to reduce sideways circular movement
	Vector3 adjustedAngels;         // adjusts to reduce sideways circular movement
	bool strafingBool;
	float strafingDistance = 10;
	Vector3 strafingDirection;      // does not change when strafing.
	Vector3 strafingCrossProduct;   // does not change when strafing.
	Vector2 strafingAngles;         // does not change when strafing.

	// ship movement parameters
	float maxForwardThrust = 10;
	float maxBackwardThrust = 3;
	float maxLateralThrust = 3;
	float maxVerticalThrust = 3;
	float maxPitchTorque = 1;
	float maxYawTorque = 1;
	float maxRollTorque = 1;

	// PID objects - totalGain, p, i, d, maxInt, min, max
	// position
	controllerSystem.PID forwardPID, lateralPID, verticalPID, pitchPID, yawPID, rollPID;
	// velocity
	controllerSystem.PID forwardVelocityPID, lateralVelocityPID, verticalVelocityPID, pitchVelocityPID, yawVelocityPID, rollVelocityPID;
	// Adjust their values in the start() method.


	// misc
	public Rigidbody rb;
	float dt;


	// Use this for initialization
	void Start()
	{

		targetLocation = transform.position;
		rb = GetComponent<Rigidbody>();
		targetAngles = new Vector2(0, 0);
		strafingBool = true;
		strafingDirection = (targetLocation - transform.position).normalized;

		// initiate PIDS -							totalGain, p, i, d, maxInt
		// position
		forwardPID = new controllerSystem.PID(2, 25f, 0.1f, 10f, 1);
		forwardVelocityPID = new controllerSystem.PID(50, 0.1f, 0, 0.1f, 1);
		lateralPID = new controllerSystem.PID(1, 5, 1, 0, 3);
		lateralVelocityPID = new controllerSystem.PID(0.1f, 1, 1, 1, 1);
		verticalPID = new controllerSystem.PID(1, 5, 1, 1, 3);
		verticalVelocityPID = new controllerSystem.PID(0.1f, 1, 1, 1, 1);
		// initiate PIDS -							totalGain, p, i, d, maxInt
		// direction
		pitchPID =			new controllerSystem.PID(0.01f, 0.1f, 0.5f, 5, 1);
		pitchVelocityPID =	new controllerSystem.PID(1, 0.1f, 0, 1, 1);
		yawPID =			new controllerSystem.PID(0.01f, 0.1f, 0.5f, 5, 1);
		yawVelocityPID =	new controllerSystem.PID(1, 0.1f, 0, 1, 1);
		rollPID =			new controllerSystem.PID(0.01f, 0.1f, 0.5f, 5, 1);
		rollVelocityPID =	new controllerSystem.PID(1, 0.1f, 0, 1, 1);
	}

	// Update is called once per frame
	void Update()
	{

	}

	private void FixedUpdate()
	{
		dt = Time.fixedDeltaTime;

		updateDirections(); // update information on target and bearings.
		//applyThrust();
		applyTorque();

		//Debug.DrawRay(transform.position, transform.forward * 100, Color.green, 0f, false);
		//Debug.DrawRay(transform.position, transform.right * 100, Color.green, 0f, false);
		//Debug.DrawRay(transform.position, transform.up * 100, Color.green, 0f, false);

	}

	public void setMovementTarget(Vector3 target)
	{
		targetLocation = target;
		strafingBool = false;
	}

	private void updateDirections()
	{
		targetDirection = (targetLocation - transform.position).normalized; // Global space
		targetDistance = (targetLocation - transform.position).magnitude;
		bearing = transform.InverseTransformDirection(targetDirection);     // Local space
		velocity = transform.InverseTransformDirection(rb.velocity);        // Local space

		// change this for 3d.
		targetCrossProduct = Vector3.Cross(transform.forward, targetDirection);     // Global space

		strafingCrossProduct = Vector3.Cross(transform.forward, strafingDirection); // Global space

		Vector3 tempVector = transform.InverseTransformDirection(targetDirection);  // Local space

		//Get target angles
		if (true)
		{
			if (targetCrossProduct.y < 0)
				targetAngles.x = -(Vector3.Angle(transform.InverseTransformDirection(transform.forward), new Vector3(tempVector.x, 0, tempVector.z)));  // Local space
			else
				targetAngles.x = Vector3.Angle(transform.InverseTransformDirection(transform.forward), new Vector3(tempVector.x, 0, tempVector.z));     // Local space

			if (targetCrossProduct.x < 0)
				targetAngles.y = -(Vector3.Angle(transform.InverseTransformDirection(transform.forward), targetDirection));     // Local space
			else
				targetAngles.y = Vector3.Angle(transform.InverseTransformDirection(transform.forward), targetDirection);        // Local space
		}

		//Get strafing angles
		if (true)
		{
			if (targetDistance < strafingDistance)
			{
				strafingBool = true;
			}
			else if (targetDistance > strafingDistance)

				if (!strafingBool)
				{
					strafingDirection = targetDirection;
					tempVector = transform.InverseTransformDirection(strafingDirection);  // Local space
				}
				else if (targetDistance > strafingDistance)
					strafingBool = false;

			if (strafingCrossProduct.y < 0)
				strafingAngles.x = -(Vector3.Angle(transform.InverseTransformDirection(transform.forward), new Vector3(tempVector.x, 0, tempVector.z)));
			else
				strafingAngles.x = Vector3.Angle(transform.InverseTransformDirection(transform.forward), new Vector3(tempVector.x, 0, tempVector.z));

			if (strafingCrossProduct.x < 0)
				strafingAngles.y = -(Vector3.Angle(transform.InverseTransformDirection(transform.forward), strafingDirection));
			else
				strafingAngles.y = Vector3.Angle(transform.InverseTransformDirection(transform.forward), strafingDirection);
		}

		//Get adjusted bearing angles
		if (targetDistance > rb.velocity.magnitude)
		{
			adjustedBearings = (targetDirection * targetDistance - rb.velocity * 0.5f);
			Debug.DrawRay(transform.position, adjustedBearings, Color.blue, 0f, false);
			adjustedCross = Vector3.Cross(transform.forward, adjustedBearings);     // Global space
			adjustedBearings = transform.InverseTransformDirection(adjustedBearings);

			if (adjustedCross.y < 0)
				adjustedAngels.x = -(Vector3.Angle(transform.InverseTransformDirection(transform.forward), new Vector3(adjustedBearings.x, 0, adjustedBearings.z)));  // Local space
			else
				adjustedAngels.x = Vector3.Angle(transform.InverseTransformDirection(transform.forward), new Vector3(adjustedBearings.x, 0, adjustedBearings.z));     // Local space

			if (adjustedCross.x < 0)
				adjustedAngels.y = -(Vector3.Angle(transform.InverseTransformDirection(transform.forward), adjustedBearings));     // Local space
			else
				adjustedAngels.y = Vector3.Angle(transform.InverseTransformDirection(transform.forward), adjustedBearings);        // Local space
		}
		else
		{
			adjustedBearings = (targetDirection * targetDistance);
			adjustedAngels = targetAngles;
		}
		Debug.DrawRay(transform.position, targetDirection * targetDistance, Color.red, 0f, false);


	}

	private void applyThrust()
	{
		Vector3 force = new Vector3(0, 0, 0);





		force.z += forwardPID.getOutput(bearing.z, dt);
		force.z += forwardVelocityPID.getOutput(-velocity.z, dt);

		if (force.z < -maxBackwardThrust)
			force.z = -maxBackwardThrust;
		else if (force.z > maxForwardThrust)
			force.z = maxForwardThrust;
		force.y += verticalPID.getOutput(bearing.y, dt);
		force.y += verticalVelocityPID.getOutput(-velocity.y, dt);
		if (force.y < -maxVerticalThrust)
			force.y = -maxVerticalThrust;
		else if (force.y > maxVerticalThrust)
			force.y = maxVerticalThrust;
		force.x += lateralPID.getOutput(bearing.x, dt);
		force.x += lateralVelocityPID.getOutput(-velocity.x, dt);
		if (force.x < -maxVerticalThrust)
			force.x = -maxVerticalThrust;
		else if (force.x > maxVerticalThrust)
			force.x = maxVerticalThrust;

		rb.AddForce(transform.forward * force.z + transform.up * force.y + transform.right * force.x);




	}

	private void applyTorque()
	{
		Vector3 torque = new Vector3(0, 0, 0);

		if (strafingBool)
		{
			torque.y += yawPID.getOutput(strafingAngles.x, dt);
			torque.y += yawVelocityPID.getOutput(-rb.angularVelocity.y, dt);

			torque.x += pitchPID.getOutput(strafingAngles.y, dt);
			torque.x += pitchVelocityPID.getOutput(-rb.angularVelocity.x, dt);
		}
		else
		{
			torque.y += yawPID.getOutput(adjustedAngels.x, dt);
			torque.y += yawVelocityPID.getOutput(-rb.angularVelocity.y, dt);

			torque.x += pitchPID.getOutput(strafingAngles.y, dt);
			torque.x += pitchVelocityPID.getOutput(-rb.angularVelocity.x, dt);
		}

		print(rb.angularVelocity.x);

		if (torque.y < -maxYawTorque)
			torque.y = -maxYawTorque;
		else if (torque.y > maxYawTorque)
			torque.y = maxYawTorque;

		if (torque.x < -maxPitchTorque)
			torque.x = -maxPitchTorque;
		else if (torque.x > maxPitchTorque)
			torque.x = maxPitchTorque;


		rb.AddTorque(transform.TransformDirection(torque));

	}
}
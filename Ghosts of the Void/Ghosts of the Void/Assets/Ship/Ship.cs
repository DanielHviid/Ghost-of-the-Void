using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ship : MonoBehaviour {

	// global vectors
	public Vector3 targetPosition;
	public Rigidbody rb;
	Vector3 targetDirection;
	Vector3 direction;

	// local vectors
	Quaternion lookRotation;
	Vector3 velocity;
	Vector3 bearing;

	// information
	bool targetOn;
	bool strafingOn;
	float distance;
	float targetAngle;
	float bearingAngle;
	Vector3 cross;

	// constants
	float strafingDistance = 5;
	float cruiseDistance;
	float deadzone = 0f;
	float rotationDeadzone = 0.5f;

	// Forward PID controller attributes
	float f_pidGain = 1;
	float f_pGain = 50;
	float f_iGain = 1;
	float f_I = 0;
	float f_IReverse = 0;
	float f_iMax = 10;
	float f_dGain = 50;
	float f_maxThrust = 10;
	float f_maxReverseThrust = 3f;
	float f_maxVelocity = 10;
	float f_maxReverseVelocity = 3;


	// Lateral PID controller attributes
	float l_pidGain = 1;
	float l_pGain = 1f;
	float l_iGain = 1f;
	float l_I = 0;
	float l_iMax = 25;
	float l_dGain = 25;
	float l_maxThrust = 3;
	float l_maxVelocity = 3;

	// Yaw PID controller attributes
	float y_pidGain = 1f;
	float y_pGain = 0.1f;
	float y_iGain = 0.1f;
	float y_I = 0;
	float y_iMax = 10f;
	float y_dGain = 10;
	float y_maxThrust = 0.5f;
	float y_maxVelocity = 0.5f;


	// Use this for initialization
	void Start() {

		targetPosition = transform.position;
		targetOn = false;
		strafingOn = false;
		rb = GetComponent<Rigidbody>();

		cruiseDistance = (f_maxThrust + f_dGain * f_maxVelocity - f_iMax) / f_pGain;
		print(cruiseDistance);
	}

	// Update is called once per frame
	void Update() {


	}
	
	private void FixedUpdate()
	{
		

		if (targetOn)
		{
			updateBearing();

			
			moveToTarget();

		}
		else
		{
			stop();
		}
		
		if (targetAngle > rotationDeadzone || targetAngle < -rotationDeadzone)
			lookAtTarget();
		else
			stopRotation();

		Debug.DrawRay(transform.position, rb.velocity, Color.black, 0f, false);
		Debug.DrawRay(transform.position, targetDirection * distance, Color.red, 0f, false);
		Debug.DrawRay(transform.position, direction * distance - rb.velocity, Color.blue, 0f, false);


	}

	public void setTarget(Vector3 target)
	{
		targetPosition = target;
		targetOn = true;
	}
	
	void updateBearing()
	{
		
		targetDirection = (targetPosition - transform.position);
		distance = targetDirection.magnitude;
		targetDirection.y = 0;
		targetDirection = targetDirection.normalized;
		
		bearing = transform.InverseTransformDirection(targetDirection);
		velocity = transform.InverseTransformDirection(rb.velocity);

		if (distance < strafingDistance)
		{
			strafingOn = true;

		}
		else
		{
			strafingOn = false;
			direction = targetDirection;
		}



		// change this for 3d.
		cross = Vector3.Cross(transform.forward, direction);
		cross.x = 0;
		cross.z = 0;

		if (cross.y < 0)
			targetAngle = -(Vector3.Angle(transform.forward, targetDirection));
		else
			targetAngle = Vector3.Angle(transform.forward, targetDirection);

		if (distance > cruiseDistance && distance > rb.velocity.magnitude)
		{
			if (cross.y > 0)
				bearingAngle = Vector3.Angle(transform.forward, direction * distance - rb.velocity) - 90;
			else
				bearingAngle = Vector3.Angle(transform.forward, direction * distance - rb.velocity) - 90;

		}
		else
		{
			bearingAngle = targetAngle;
		}

	}
	
	void moveToTarget()
	{
		float throttlePID = 0;
		float lateralPID = 0;

		

		if (targetAngle > 90 || targetAngle < -90) // backwards
		{
			throttlePID = ForcePID(f_pidGain, f_pGain, f_iGain, f_dGain, ref f_I, f_iMax, bearing.z, velocity.z, f_maxReverseThrust, f_maxThrust, f_maxVelocity, f_maxVelocity);
			f_IReverse = -f_I;
		}
		else                                         // forwards
		{
			if (distance > cruiseDistance)
			{
				throttlePID = ForcePID(f_pidGain, f_pGain, f_iGain, f_dGain, ref f_I, f_iMax, bearing.magnitude, velocity.z, f_maxReverseThrust, f_maxThrust, f_maxVelocity, f_maxVelocity);
				f_IReverse = -f_I;
			}
			else
				throttlePID = ForcePID(f_pidGain, f_pGain, f_iGain, f_dGain, ref f_IReverse, f_iMax, bearing.z, velocity.z, f_maxThrust, f_maxReverseThrust, f_maxVelocity, f_maxVelocity);
			f_I = -f_IReverse;
		}

		lateralPID =  ForcePID(l_pidGain, l_pGain, l_iGain, l_dGain, ref l_I, l_iMax, bearing.x, velocity.x, l_maxThrust, l_maxVelocity);


		Debug.DrawRay(transform.position, transform.forward * throttlePID, Color.green, 0f, false);
		Debug.DrawRay(transform.position, transform.right * lateralPID, Color.blue, 0f, false);

		rb.AddForce(transform.forward * throttlePID);
		rb.AddForce(transform.right * lateralPID);
		
		

	}

	void lookAtTarget()
	{
		
		float torque;
		
		

		torque = ForcePID(y_pidGain, y_pGain, y_iGain, y_dGain, ref y_I, y_iMax, targetAngle, rb.angularVelocity.y, y_maxThrust, y_maxVelocity);

		rb.AddTorque((targetAngle * new Vector3(0,cross.y,0)).normalized * torque);
		
	}

	void stop()
	{
		rb.AddForce(-rb.velocity * 0.9f);
	}

	void stopRotation()
	{
		rb.AddTorque(rb.angularVelocity * 0.9f);
	}

	float ForcePID(float gain, float pGain, float iGain, float dGain, ref float integral, float iMax, float distance, float velocity, float maxThrust, float maxVelocity)
	{
		float thrust = PID(gain, pGain, iGain, dGain, ref integral, iMax, distance, velocity, maxThrust);

		
		if (thrust + velocity < maxVelocity || thrust + velocity > -maxVelocity)
		{
			
			return thrust;
		}
		
		return maxVelocity - velocity;
	}

	float ForcePID(float gain, float pGain, float iGain, float dGain, ref float integral, float iMax, float distance, float velocity, float maxThrust, float maxReverseThrust, float maxVelocity, float maxReverseVelocity)
	{
		float thrust = PID(gain, pGain, iGain, dGain, ref integral, iMax, distance, velocity, maxThrust);
		
		if (thrust + velocity < maxVelocity && thrust + velocity > -maxVelocity)
		{
			return thrust;
		}
		return maxVelocity - velocity;
	}

	float PID(float gain, float pGain, float iGain, float dGain, ref float integral, float iMax, float distance, float velocity, float maxThrust)
	{
		float pid;

		integral += iGain * distance;

		if (integral > iMax)
		{
			integral = iMax;
		}
		else if (integral < -iMax)
		{
			integral = -iMax;
		}



		pid = gain * (pGain * distance + integral - (dGain * velocity));


		if (pid < -maxThrust)
		{
			return -maxThrust;

		}
		if (pid > maxThrust)
		{
			return maxThrust;
		}


		return pid;
	}
	
	float PID(float gain, float pGain, float iGain, float dGain, ref float integral, float iMax, float distance, float velocity, float maxThrust, float maxReverseThrust, float maxVelocity, float maxReverseVelocity)
	{
		float pid;

		integral += iGain * distance;

		if (integral > iMax)
		{
			integral = iMax;
		}
		else if (integral < -iMax)
		{
			integral = -iMax;
		}



		pid = gain * (pGain * distance + integral - (dGain * velocity));


		if (pid < -maxThrust)
		{
			return -maxThrust;

		}
		if (pid > maxThrust)
		{
			return maxThrust;
		}


		return pid;
	}

}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Gimball : Ship {

	Vector3 targetLocation;
	Vector3 targetDirection;

	float maxAngle = 80;
	float angle;
	Vector3 origin;// parent space

	// Use this for initialization
	void Start () {
		origin = transform.parent.InverseTransformDirection(transform.forward); // parent space
		targetLocation = new Vector3(25, 1, 25);
	}
	
	// Update is called once per frame
	void Update () {

	}

	private void FixedUpdate()
	{
		updateDirections();
		rotate();
	}

	void updateDirections()
	{
		targetDirection = (targetLocation - transform.position).normalized;

		angle = Vector3.Angle(transform.parent.TransformDirection(origin), targetDirection);
		Debug.DrawRay(transform.position, transform.forward * 100, Color.green, 0f, false);
	}
	void rotate()
	{
		if (angle < maxAngle)
		{
			//transform.LookAt((targetLocation));
			rotateToPosition(targetDirection);
			//transform.rotation = Quaternion.Slerp(Quaternion.LookRotation(transform.forward, transform.parent.up), Quaternion.LookRotation(targetDirection, transform.parent.up), Time.time * 0.01f);
		}
		else
		{
			rotateToPosition(transform.parent.TransformDirection(origin));
			//transform.LookAt(transform.position + transform.parent.TransformDirection(origin));
		}
	}

	void rotateToPosition(Vector3 target)
	{
		transform.rotation = Quaternion.Slerp(Quaternion.LookRotation(transform.forward, transform.parent.up), Quaternion.LookRotation(target, transform.parent.up), Time.time * 0.01f);
	}

	void rotateToDirection(Vector3 Direction)
	{
		transform.rotation = Quaternion.Slerp(Quaternion.LookRotation(transform.forward, transform.parent.up), Quaternion.LookRotation(Direction, transform.parent.up), Time.time * 0.01f);
	}
	
}

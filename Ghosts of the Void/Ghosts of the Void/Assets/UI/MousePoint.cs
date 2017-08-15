using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MousePoint : MonoBehaviour {

    RaycastHit hit;
	
	// Update is called once per frame
	void Update ()
    {

        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

        if (Input.GetMouseButtonDown(0))
        {
            if (Physics.Raycast(ray, out hit, 1000))
            {

                if (hit.collider.name == "Background")
                {
                    Vector3 target = hit.point;
					GameObject.Find("Ship").GetComponent<ShipMovement>().setMovementTarget(new Vector3(target.x,1,target.z));
					print("New target location");
                }
            }
        }
		
	}

    
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ApplyTorque : MonoBehaviour {
    public GameObject p1;
    public GameObject j1;
    public Rigidbody r1;

	// Use this for initialization
	void Start () {
        r1 = p1.gameObject.GetComponent<Rigidbody>();
	}
	
	// Update is called once per frame
	void Update () {
        if (Input.GetKey("s"))
        {
            r1.AddForceAtPosition(new Vector3(1,0,0),j1.transform.position,ForceMode.Force);
        }
		
	}
}

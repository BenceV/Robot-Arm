using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ManualControl : MonoBehaviour {
    public HingeJoint hinge0;
    public HingeJoint hinge1;
    public HingeJoint hinge2;

    // Use this for initialization
    void Start () {
    }
	
	// Update is called once per frame
	void Update () {
        var motor0 = hinge0.motor;
        var motor1 = hinge1.motor;
        var motor2 = hinge2.motor;
        if (Input.GetKey("1"))
        {
            // Make the hinge motor rotate with 150 degrees per second and a strong force.

            motor0.force = 10;
            motor0.targetVelocity = 150;
            motor0.freeSpin = false;
            hinge0.motor = motor0;
            hinge0.useMotor = true;
        }
        else if (Input.GetKey("2"))
        {
            // Make the hinge motor rotate with 150 degrees per second and a strong force.

            motor0.force = 10;
            motor0.targetVelocity = -150;
            motor0.freeSpin = false;
            hinge0.motor = motor0;
            hinge0.useMotor = true;
        }
        else if (Input.GetKey("3"))
        {
            // Make the hinge motor rotate with 150 degrees per second and a strong force.

            motor1.force = 10;
            motor1.targetVelocity = -150;
            motor1.freeSpin = false;
            hinge1.motor = motor1;
            hinge1.useMotor = true;
        }
        else if (Input.GetKey("4"))
        {
            // Make the hinge motor rotate with 150 degrees per second and a strong force.

            motor1.force = 10;
            motor1.targetVelocity = -150;
            motor1.freeSpin = false;
            hinge1.motor = motor1;
            hinge1.useMotor = true;
        }
        else if (Input.GetKey("5"))
        {
            // Make the hinge motor rotate with 150 degrees per second and a strong force.

            motor2.force = 10;
            motor2.targetVelocity = -150;
            motor2.freeSpin = false;
            hinge2.motor = motor2;
            hinge2.useMotor = true;
        }
        else if (Input.GetKey("6"))
        {
            // Make the hinge motor rotate with 150 degrees per second and a strong force.

            motor2.force = 10;
            motor2.targetVelocity = -150;
            motor2.freeSpin = false;
            hinge2.motor = motor2;
            hinge2.useMotor = true;
        }
        else
        {
            hinge0.useMotor = false;
            hinge1.useMotor = false;
            hinge2.useMotor = false;

        }
    }
}

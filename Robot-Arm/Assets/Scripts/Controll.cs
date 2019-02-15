using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;


public class Controll : MonoBehaviour
{   
    //Simpler
    //Real Arm
    public GameObject j0;
    public GameObject j1;
    public GameObject j2;
    public GameObject ee;
    //Real Joints
    public GameObject p0;
    public GameObject p1;
    public GameObject p2;

    //Target
    public GameObject target;

    //The Test Arm
    public GameObject p0t;
    public GameObject p1t;
    public GameObject p2t;
    //The Test Joints
    public GameObject j0t;
    public GameObject j1t;
    public GameObject j2t;
    public GameObject eet;
    
    //Arm rotation axes
    private Vector3[] axes;
    //Previous joint angles
    private float[] prev_joint_angles;
    //Hinges
    public HingeJoint hinge0;
    public HingeJoint hinge1;
    public HingeJoint hinge2;

    //PD
    float[] P;
    float[] D;

    // Use this for initialization
    void Start()
    {
        axes = new Vector3[3];
        axes[0] = new Vector3(0, 1, 0);
        axes[1] = new Vector3(1, 0, 0);
        axes[2] = new Vector3(1, 0, 0);

        prev_joint_angles = new float[] { 0,0,0};

        P = new float[] { 5000, 100, 100 };
        D = new float[] { 100, 175, 75 };
    }

    // Update is called once per frame
    void Update()
    {
        //Motor
        var motor0 = hinge0.motor;
        var motor1 = hinge1.motor;
        var motor2 = hinge2.motor;
        //Angles
        float[] angles = GetAngles(axes);
        print("Angles: "+Math.Round(angles[0] * Mathf.Rad2Deg, 2)+", "+ Math.Round(angles[1] * Mathf.Rad2Deg, 2) + ", " + Math.Round(angles[2] * Mathf.Rad2Deg, 2));
        //EE position
        print("Actual position: " + Math.Round(ee.transform.position.x, 2)+", " + Math.Round(ee.transform.position.y, 2)+", " + Math.Round(ee.transform.position.z, 2));
        //IK
        float[] ik = IK(angles,new float[] {target.transform.position.x, target.transform.position.y, target.transform.position.z});
        //Desired angles
        float[] desired_angles = new float[] { angles[0] + ik[0], angles[1] + ik[1], angles[2] + ik[2] };
        //Current angle velocities
        float[] angle_velocities = new float[] {AngleNormalize(angles[0]-prev_joint_angles[0])/Time.deltaTime,AngleNormalize(angles[1]-prev_joint_angles[1]) / Time.deltaTime, AngleNormalize(angles[2]-prev_joint_angles[2]) / Time.deltaTime };
        print(angle_velocities[0] +", "+angle_velocities[1]+", "+angle_velocities[2]);
        //PD Control

        //float[] P = new float[] { 5000, 100, 100 };
        //float[] D = new float[] { 100, 1000, 100 };
        
        if (Input.GetKey("1")) {
            P[0] = P[0] + 10;
        }
        if (Input.GetKey("2"))
        {
            P[1] = P[1] + 10;
        }
        if (Input.GetKey("3"))
        {
            P[2] = P[2] + 10;
        }

        if (Input.GetKey("q"))
        {
            D[0] = D[0] + 10;
        }
        if (Input.GetKey("w"))
        {
            D[1] = D[1] + 10;
        }
        if (Input.GetKey("e"))
        {
            D[2] = D[2] + 10;
        }
        print("P gains: " + P[0] + ", " + P[1] + "," + P[2]);
        print("D gains: " + D[0] + ", " + D[1] + "," + D[2]);

        float[] torques = js_pd_control(D,P,angles,angle_velocities,desired_angles);
        print(torques[0] * (float)0.08 + ", "+torques[1] * (float)0.08 + ", "+torques[2] * (float)0.08);


        //Apply torques
        motor0.force = Math.Abs(torques[0] * (float)0.1);
        motor0.targetVelocity = Math.Sign(torques[0])*150;
        motor0.freeSpin = false;
        hinge0.motor = motor0;
        hinge0.useMotor = true;


        motor1.force = Math.Abs(torques[1] * (float)0.5);
        motor1.targetVelocity = Math.Sign(torques[1])*150;
        motor1.freeSpin = false;
        hinge1.motor = motor1;
        hinge1.useMotor = true;

        motor2.force = Math.Abs(torques[2] * (float)0.5);
        motor2.targetVelocity = Math.Sign(torques[2])*150;
        motor2.freeSpin = false;
        hinge2.motor = motor2;
        hinge2.useMotor = true;

        //p0.gameObject.transform.Rotate(axes[0], ik[0] * (float)0.08 * Mathf.Rad2Deg);
        //p1.gameObject.transform.Rotate(axes[1], ik[1] * (float)0.04 * Mathf.Rad2Deg);
        //p2.gameObject.transform.Rotate(axes[2], ik[2] * (float)0.04 * Mathf.Rad2Deg);
        //print("Inverse Kinematics: "+ ik[0]+", "+ik[1] + ", "+ik[2]);
        print("Distance:" + Vector3.Distance( p1.transform.position,target.transform.position));

        //Set new prev_joint_angles
        prev_joint_angles = angles;

    }

    float[,] FK(float[] angles) {
       //Forward Kinematics
       //Get Homogenouous Transformation Matrices
        var t0 = GetTransformationMatrix(angles[0], "y");
        var t1 = GetTransformationMatrix(angles[1], "x");
        var t2 = GetTransformationMatrix(angles[2], "x");

        var k = Multiply4x4(t0,Multiply4x4(t1, t2));
        
        print("Forward kinematics: " + (Math.Round(k[0, 3], 2)) +", "+ (Math.Round(k[1, 3], 2)) + ", " + (Math.Round(k[2, 3], 2)));
        //ShowMatrix4x4(k);
        return k;

    }

    float[] IK(float[] current_angles,float[] desired_position) {
        float[,] fk = FK(current_angles);
        float[] curr_pos = new float[] { fk[0, 3], fk[1, 3], fk[2, 3] };

        //float[] curr_pos = new float[] { ee.transform.position.x, ee.transform.position.y, ee.transform.position.z };
        float[] pos_error = Negative3D(desired_position,curr_pos);

        float[,] Jac = Jacobian(current_angles);
        float[,] jac = new float[3, 3];
        jac[0, 0] = Jac[0, 0];
        jac[1, 0] = Jac[1, 0];
        jac[2, 0] = Jac[2, 0];

        jac[0, 1] = Jac[0, 1];
        jac[1, 1] = Jac[1, 1];
        jac[2, 1] = Jac[2, 1];

        jac[0, 2] = Jac[0, 2];
        jac[1, 2] = Jac[1, 2];
        jac[2, 2] = Jac[2, 2];

        float[,] jac_inv = Transpose3x3(jac);

        float[] q_dot = Multipl3x1(jac_inv,pos_error);

        return q_dot;
    }

    float[,] Jacobian(float[] angles) {

        float[,] jacobian = new float[4,3];

        var j1_transform = GetTransformationMatrix(angles[0], "y");
        var j2_transform = GetTransformationMatrix(angles[1], "x");
        var j3_transform = GetTransformationMatrix(angles[2], "x");

        var total_transform = Multiply4x4(j1_transform, Multiply4x4(j2_transform, j3_transform));

        float[] z1  = new float[] { 1, 0, 0 , 1};
        float[] z2s = new float[] { 0, 1, 0};

        float[] ee = new float[3];
        ee[0] = total_transform[0, 3];
        ee[1] = total_transform[1, 3];
        ee[2] = total_transform[2, 3];

        //ee[0] = this.ee.transform.position.x;
        //ee[1] = this.ee.transform.position.y;
        //ee[2] = this.ee.transform.position.z;

        float[] j3_pos = new float[3];
        j3_pos[0] = Multiply4x4(j1_transform, j2_transform)[0, 3];
        j3_pos[1] = Multiply4x4(j1_transform, j2_transform)[1, 3];
        j3_pos[2] = Multiply4x4(j1_transform, j2_transform)[2, 3];

        float[] j2_pos = new float[3];
        j2_pos[0] = j1_transform[0, 3];
        j2_pos[1] = j1_transform[1, 3];
        j2_pos[2] = j1_transform[2, 3];

        float[] j1_pos = new float[3];
        j1_pos[0] = 0;
        j1_pos[1] = 0;
        j1_pos[2] = 0;

        //Row1 of the jacobian
        float[] jrow1 = CrossProduct3D(z2s,Negative3D(ee,j1_pos)); 
        jacobian[0, 0] = jrow1[0];
        jacobian[1, 0] = jrow1[1];
        jacobian[2, 0] = jrow1[2];

        //Row2 of the jacobian
        float[] t2 = Multiply4x1(j1_transform,z1);
        float[] t2s = new float[3] {t2[0],t2[1],t2[2]};
        float[] jrow2 = CrossProduct3D(t2s, Negative3D(ee, j2_pos));
        jacobian[0, 1] = jrow2[0];
        jacobian[1, 1] = jrow2[1];
        jacobian[2, 1] = jrow2[2];

        //Row3 of the jacobian
        float[] t3 = Multiply4x1(Multiply4x4(j1_transform,j2_transform), z1);
        float[] t3s = new float[3] { t3[0], t3[1], t3[2] };
        float[] jrow3 = CrossProduct3D(t3s, Negative3D(ee, j3_pos));
        jacobian[0, 2] = jrow3[0];
        jacobian[1, 2] = jrow3[1];
        jacobian[2, 2] = jrow3[2];

        jacobian[3, 0] = 1;
        jacobian[3, 0] = 1;
        jacobian[3, 0] = 1;

        //ShowMatrix4x3(jacobian);
        return jacobian;
    }

    void ShowMatrix4x4(float[,] result) {
        print(result[0, 0] + " " + result[0, 1] + " " + result[0, 2] + " " + result[0, 3]);
        print(result[1, 0] + " " + result[1, 1] + " " + result[1, 2] + " " + result[1, 3]);
        print(result[2, 0] + " " + result[2, 1] + " " + result[2, 2] + " " + result[2, 3]);
        print(result[3, 0] + " " + result[3, 1] + " " + result[3, 2] + " " + result[3, 3]);
    }

    void ShowMatrix4x3(float[,] result)
    {
        print(Math.Round(result[0, 0],2) + " " + Math.Round(result[0, 1],2) + " " + Math.Round(result[0, 2],2) );
        print(Math.Round(result[1, 0],2) + " " + Math.Round(result[1, 1],2) + " " + Math.Round(result[1, 2],2) );
        print(Math.Round(result[2, 0],2) + " " + Math.Round(result[2, 1],2) + " " + Math.Round(result[2, 2],2) );
        print(Math.Round(result[3, 0],2) + " " + Math.Round(result[3, 1],2) + " " + Math.Round(result[3, 2],2) );
    }

    float[,] GetTransformationMatrix(float angle, string axis) {
        float[,] R = new float[4, 4];
        float[,] T = new float[4, 4];

        //The Rotation
        if (axis=="x") {
            R[0, 0] = 1;
            R[0, 1] = 0;
            R[0, 2] = 0;
            R[0, 3] = 0;

            R[1, 0] = 0;
            R[1, 1] = Mathf.Cos(angle);
            R[1, 2] = -Mathf.Sin(angle);
            R[1, 3] = 0;

            R[2, 0] = 0;
            R[2, 1] = Mathf.Sin(angle);
            R[2, 2] = Mathf.Cos(angle);
            R[2, 3] = 0;

            //The Translation
            T[0, 0] = 1;
            T[0, 1] = 0;
            T[0, 2] = 0;
            T[0, 3] = 0;

            T[1, 0] = 0;
            T[1, 1] = 1;
            T[1, 2] = 0;
            T[1, 3] = 3;

            T[2, 0] = 0;
            T[2, 1] = 0;
            T[2, 2] = 1;
            T[2, 3] = 0;

            T[3, 0] = 0;
            T[3, 1] = 0;
            T[3, 2] = 0;
            T[3, 3] = 1;

        }
        if (axis == "y")
        {
            R[0, 0] = Mathf.Cos(angle);
            R[0, 1] = 0;
            R[0, 2] = Mathf.Sin(angle);
            R[0, 3] = 0;

            R[1, 0] = 0;
            R[1, 1] = 1;
            R[1, 2] = 0;
            R[1, 3] = 0;

            R[2, 0] = -Mathf.Sin(angle);
            R[2, 1] = 0;
            R[2, 2] = Mathf.Cos(angle);
            R[2, 3] = 0;

            //The Translation
            T[0, 0] = 1;
            T[0, 1] = 0;
            T[0, 2] = 0;
            T[0, 3] = 0;

            T[1, 0] = 0;
            T[1, 1] = 1;
            T[1, 2] = 0;
            T[1, 3] = 3;

            T[2, 0] = 0;
            T[2, 1] = 0;
            T[2, 2] = 1;
            T[2, 3] = 0;

            T[3, 0] = 0;
            T[3, 1] = 0;
            T[3, 2] = 0;
            T[3, 3] = 1;

        }
        if (axis == "z")
        {
            R[0, 0] = Mathf.Cos(angle);
            R[0, 1] = -Mathf.Sin(angle);
            R[0, 2] = 0;
            R[0, 3] = 0;

            R[1, 0] = Mathf.Sin(angle);
            R[1, 1] = Mathf.Cos(angle);
            R[1, 2] = 0;
            R[1, 3] = 0;

            R[2, 0] = 0;
            R[2, 1] = 0;
            R[2, 2] = 1;
            R[2, 3] = 0;

            //The Translation
            T[0, 0] = 1;
            T[0, 1] = 0;
            T[0, 2] = 0;
            T[0, 3] = 0;

            T[1, 0] = 0;
            T[1, 1] = 1;
            T[1, 2] = 0;
            T[1, 3] = 3;

            T[2, 0] = 0;
            T[2, 1] = 0;
            T[2, 2] = 1;
            T[2, 3] = 0;

            T[3, 0] = 0;
            T[3, 1] = 0;
            T[3, 2] = 0;
            T[3, 3] = 1;


        }


        R[3, 0] = 0;
        R[3, 1] = 0;
        R[3, 2] = 0;
        R[3, 3] = 1;

        


        //print("Transformation");
        //ShowMatrix4x4(T);
        //print("Rotation");
        //ShowMatrix4x4(R);
        var result = Multiply4x4(R, T);
        //print("Result");
        //ShowMatrix4x4(result);

        return result;
    }

    float[] GetAngles(Vector3[] axes) {
        //Instantiate Test Arm
        p0t = Instantiate(p0);
        p1t = p0t.transform.GetChild(2).gameObject;
        p2t = p1t.transform.GetChild(2).gameObject;
        //Instantiate Test Joints
        j0t = Instantiate(j0);
        j1t = p0t.transform.GetChild(2).gameObject;
        j2t = p1t.transform.GetChild(2).gameObject;
        eet = p2t.transform.GetChild(1).gameObject;

        float[] angles = new float[3];

        //Joint0
        if (p0t.transform.rotation.eulerAngles != new Vector3(0, 0, 0))
        {
            //float angle0 = Mathf.Atan2(j1t.transform.position.z - j0t.transform.position.z, j1t.transform.position.x - j0t.transform.position.x);
            float angle0 = AngleNormalize(p0t.transform.rotation.eulerAngles.y*Mathf.Deg2Rad);
            angles[0] = angle0;
            //Rotate the test arm by the first angle
            p0t.transform.Rotate(axes[0], -angle0 * Mathf.Rad2Deg);
        }

        //Joint1
        if (p1t.transform.rotation.eulerAngles != new Vector3(0, 0, 0))
        {
            float angle1 = Mathf.Atan2(j2t.transform.position.z - j1t.transform.position.z, j2t.transform.position.y - j1t.transform.position.y);
            angle1 = AngleNormalize(angle1);
            angles[1] = angle1;
            //Rotate part1 by the 2nd angle
            p1t.transform.Rotate(axes[1], -angle1 * Mathf.Rad2Deg);
        }

        //Joint2
        if (p2t.transform.rotation.eulerAngles != new Vector3(0, 0, 0))
        {

            float angle2 = Mathf.Atan2(eet.transform.position.z - j2t.transform.position.z, eet.transform.position.y - j2t.transform.position.y);
            angle2 = AngleNormalize(angle2);
            angles[2] = angle2;
            //Rotate part2 by the 3rd angle
            p2t.transform.Rotate(axes[2]*-1, -angle2 * Mathf.Rad2Deg);

        }

        Destroy(p0t);
        Destroy(p1t);
        Destroy(p2t);

        Destroy(j0t);
        Destroy(j1t);
        Destroy(j2t);
        Destroy(eet);
        
        return (angles);
    }

    float AngleNormalize(float angle) {
        return (((angle + Mathf.PI) % (2 * Mathf.PI)) - Mathf.PI);
    }

    float[,] Multiply4x4(float[,] A, float[,] B) {
        float[,] R = new float[4, 4];
        R[0, 0] = A[0, 0] * B[0, 0] + A[0, 1] * B[1, 0] + A[0, 2] * B[2, 0] + A[0, 3] * B[3, 0];
        R[0, 1] = A[0, 0] * B[0, 1] + A[0, 1] * B[1, 1] + A[0, 2] * B[2, 1] + A[0, 3] * B[3, 1];
        R[0, 2] = A[0, 0] * B[0, 2] + A[0, 1] * B[1, 2] + A[0, 2] * B[2, 2] + A[0, 3] * B[3, 2];
        R[0, 3] = A[0, 0] * B[0, 3] + A[0, 1] * B[1, 3] + A[0, 2] * B[2, 3] + A[0, 3] * B[3, 3];


        R[1, 0] = A[1, 0] * B[0, 0] + A[1, 1] * B[1, 0] + A[1, 2] * B[2, 0] + A[1, 3] * B[3, 0];
        R[1, 1] = A[1, 0] * B[0, 1] + A[1, 1] * B[1, 1] + A[1, 2] * B[2, 1] + A[1, 3] * B[3, 1];
        R[1, 2] = A[1, 0] * B[0, 2] + A[1, 1] * B[1, 2] + A[1, 2] * B[2, 2] + A[1, 3] * B[3, 2];
        R[1, 3] = A[1, 0] * B[0, 3] + A[1, 1] * B[1, 3] + A[1, 2] * B[2, 3] + A[1, 3] * B[3, 3];


        R[2, 0] = A[2, 0] * B[0, 0] + A[2, 1] * B[1, 0] + A[2, 2] * B[2, 0] + A[2, 3] * B[3, 0];
        R[2, 1] = A[2, 0] * B[0, 1] + A[2, 1] * B[1, 1] + A[2, 2] * B[2, 1] + A[2, 3] * B[3, 1];
        R[2, 2] = A[2, 0] * B[0, 2] + A[2, 1] * B[1, 2] + A[2, 2] * B[2, 2] + A[2, 3] * B[3, 2];
        R[2, 3] = A[2, 0] * B[0, 3] + A[2, 1] * B[1, 3] + A[2, 2] * B[2, 3] + A[2, 3] * B[3, 3];


        R[3, 0] = A[3, 0] * B[0, 0] + A[3, 1] * B[1, 0] + A[3, 2] * B[2, 0] + A[3, 3] * B[3, 0];
        R[3, 1] = A[3, 0] * B[0, 1] + A[3, 1] * B[1, 1] + A[3, 2] * B[2, 1] + A[3, 3] * B[3, 1];
        R[3, 2] = A[3, 0] * B[0, 2] + A[3, 1] * B[1, 2] + A[3, 2] * B[2, 2] + A[3, 3] * B[3, 2];
        R[3, 3] = A[3, 0] * B[0, 3] + A[3, 1] * B[1, 3] + A[3, 2] * B[2, 3] + A[3, 3] * B[3, 3];

        return R;
    }

    float[] Multiply4x1(float[,] A, float[] b) {
        float[] R = new float[4];
        R[0] = A[0, 0] * b[0] + A[0, 1] * b[1] + A[0, 2] * b[2] + A[0, 3] * b[3];
        R[1] = A[1, 0] * b[0] + A[1, 1] * b[1] + A[1, 2] * b[2] + A[1, 3] * b[3];
        R[2] = A[2, 0] * b[0] + A[2, 1] * b[1] + A[2, 2] * b[2] + A[2, 3] * b[3];
        R[2] = A[3, 0] * b[0] + A[3, 1] * b[1] + A[3, 2] * b[2] + A[3, 3] * b[3];
        return R;
    }

    float[] Multipl3x1(float[,] A, float[] b)
    {
        float[] R = new float[3];
        R[0] = A[0, 0] * b[0] + A[0, 1] * b[1] + A[0, 2] * b[2];
        R[1] = A[1, 0] * b[0] + A[1, 1] * b[1] + A[1, 2] * b[2];
        R[2] = A[2, 0] * b[0] + A[2, 1] * b[1] + A[2, 2] * b[2];
        return R;
    }

    float[] CrossProduct3D(float[] A,float[] B) {
        float[] R =new float[3];
        R[0] = A[1] * B[2] - A[2] * B[1];
        R[1] = A[2] * B[0] - A[0] * B[2];
        R[2] = A[0] * B[1] - A[1] * B[0];
        return R;
    }

    float[] Negative3D(float[] A, float[] B) {
        float[] R = new float[3];
        R[0] = A[0] - B[0];
        R[1] = A[1] - B[1];
        R[2] = A[2] - B[2];
        return R;
    }

    float[,] TensorProduct(float[] A, float[] B) {
        float[,] R = new float[3, 3];
        R[0, 0] = A[0] * B[0];
        R[0, 1] = A[0] * B[1];
        R[0, 2] = A[0] * B[2];

        R[1, 0] = A[1] * B[0];
        R[1, 1] = A[1] * B[1];
        R[1, 2] = A[1] * B[2];

        R[2, 0] = A[2] * B[0];
        R[2, 1] = A[2] * B[1];
        R[2, 2] = A[2] * B[2];
        return R;
    }

    float[,] Transpose3x3(float[,] A) {
        float[,] R = new float[3, 3];
        R[0, 0] = A[0, 0];
        R[0, 1] = A[1, 0];
        R[0, 2] = A[2, 0];

        R[1, 0] = A[0, 1];
        R[1, 1] = A[1, 1];
        R[1, 2] = A[2, 1];

        R[2, 0] = A[0, 2];
        R[2, 1] = A[1, 2];
        R[2, 2] = A[2, 2];

        return R;
    }

    float[] js_pd_control(float[] D, float[] P, float[] current_joint_angles, float[] current_joint_velocities,float[] desired_joint_angles) {
        //Gains
        float[] P_error = new float[] {desired_joint_angles[0]-current_joint_angles[0], desired_joint_angles[1] - current_joint_angles[1], desired_joint_angles[2] - current_joint_angles[2]};
        float[] D_error = new float[] {0-current_joint_velocities[0], 0 - current_joint_velocities[1], 0 - current_joint_velocities[2] };

        float[] PD_error = new float[3];
        PD_error[0] = P[0] * P_error[0] + D[0] * D_error[0];
        PD_error[1] = P[1] * P_error[1] + D[1] * D_error[1];
        PD_error[2] = P[2] * P_error[2] + D[2] * D_error[2];
        return PD_error;
    }


}

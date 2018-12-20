using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GetJacobian : MonoBehaviour
{
    //Simpler
    public GameObject j0;
    public GameObject j1;
    public GameObject j2;
    public GameObject ee;

    public GameObject l0;
    public GameObject l1;
    public GameObject l2;

    // Use this for initialization
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        print(Quaternion.Angle(l0.transform.rotation, l1.transform.rotation));
        print(Quaternion.Angle(l1.transform.rotation, l2.transform.rotation));
        print(Quaternion.Angle(l2.transform.rotation, ee.transform.rotation));
    }
}

using System.Collections;
using UnityEngine;

public class SetConfig : MonoBehaviour {
    //Not finished Do it later
    private ArrayList links;
    private int nChild = 0;
    //Simpler
    public GameObject l0;
    public GameObject l1;
    public GameObject l2;
    public float[] rotations;



    // Use this for initialization
    void Start () {
        //Working implementation starts here
        l0.gameObject.transform.Rotate(new Vector3(0, 1, 0), rotations[0]);
        l1.gameObject.transform.Rotate(new Vector3(1, 0, 0), rotations[1]);
        l2.gameObject.transform.Rotate(new Vector3(1, 0, 0), rotations[2]);


        //Working implementation ends here
    }
    void Update()
    {


    }


    void SetupLinks(GameObject currentParent) {
        //ISSUE: Not the first child is the correct one.
        //Add currentParent to the list of links
        links.Add(currentParent);
        nChild = currentParent.transform.childCount;
        if (nChild != 0)
        {
            //If the link has a child
            GameObject nChild = currentParent.transform.GetChild(0).gameObject;

            //Recurse
            SetupLinks(nChild);
        }
        else {
            //If the link doesn't have a child
        }

    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class Rotor_Test : MonoBehaviour
{
    float[] W = new float[4];
    public int num;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Drone drone = GameObject.Find("Drone").GetComponent<Drone>();
        W[0] = (float)drone.D_W[0, 0];
        W[1] = -(float)drone.D_W[1, 0];
        W[2] = (float)drone.D_W[2, 0];
        W[3] = -(float)drone.D_W[3, 0];
        Vector3 Ro = new Vector3(0, W[num - 1], 0);
        transform.Rotate(Ro);
    }
}

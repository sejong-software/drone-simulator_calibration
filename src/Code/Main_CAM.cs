using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Main_CAM : MonoBehaviour
{
    public Transform target;
    public float dist = 1.5f;
    public float height = 3.0f;
    public float smoothRotate = 5.0f;

    private Transform Own;

    // Start is called before the first frame update
    void Start()
    {
        Own = GetComponent<Transform>();
    }

    // Update is called once per frame
    void Update()
    {
        float currYAngle = Mathf.LerpAngle(Own.eulerAngles.y, target.eulerAngles.y, smoothRotate * Time.deltaTime);
        Quaternion rot = Quaternion.Euler(0, currYAngle, 0);

        Own.position = target.position - (rot * Vector3.forward * dist) + (Vector3.up * height);

        Own.LookAt(target);
    }
}

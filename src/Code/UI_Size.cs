using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UI_Size : MonoBehaviour
{
    public float xP = 0.1f;
    public float yP = 0.1f;
    public float xS = 0.1f;
    public float yS = 0.1f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector2 myPos = new Vector2(Screen.width * xP, Screen.height * yP);
        Vector2 mySize = new Vector2(Screen.width * xS, Screen.height * yS);
        transform.gameObject.GetComponent<RectTransform>().sizeDelta = mySize;
        transform.gameObject.GetComponent<RectTransform>().position = myPos;
    }
}

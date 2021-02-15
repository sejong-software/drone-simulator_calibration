using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;


public class Text_Write : MonoBehaviour
{
    public Text txt;
    public int font_Size = 10;
    // public double Latitude, Longitude;
    // Start is called before the first frame update
    //rone drone;
    Drone drone; 

    void Start()
    {
        //drone = GameObject.Find("Drone").GetComponent<Drone>();
        drone = GameObject.Find("Drone").GetComponent<Drone>();
    }

    // Update is called once per frame
    void Update()
    {
        double[,] S_Att = new double[3, 1];
        S_Att = drone.F_Euler;
        string Output = "";
        string Title = " Drone Information";
        string AttX = " Roll  = " + (180 / Math.PI * S_Att[0, 0]).ToString("F5");
        string AttY = " Pitch = " + (180 / Math.PI * S_Att[1, 0]).ToString("F5");
        string AttZ = " Yaw   = " + (180 / Math.PI * S_Att[2, 0]).ToString("F5");
        string Att = AttX + "\r\n" + AttY + "\r\n" + AttZ;
        string PosX = " Lat   = " + (drone.F_Pos[0, 0]).ToString("F5");
        string PosY = " Lon   = " + (drone.F_Pos[1, 0]).ToString("F5");
        string PosZ = " Alt   = " + (drone.F_Pos[2, 0]).ToString("F5");
        string Pos = PosX + "\r\n" + PosY + "\r\n" + PosZ;
        string VelX = " Vn    =" + drone.F_Vel[0, 0].ToString("F5");
        string VelY = " Ve    =" + drone.F_Vel[1, 0].ToString("F5");
        string VelZ = " Vd    =" + drone.F_Vel[2, 0].ToString("F5");
        string Vel = VelX + "\r\n" + VelY + "\r\n" + VelZ;
        Output = Title + "\r\n\r\n" +  Pos + "\r\n\r\n" + Vel + "\r\n\r\n" + Att;
        //txt.fontSize = font_Size;
        txt.text = Output;
        
    }
}

using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;
using UnityEngine;
using UnityEditor;
using System.Runtime.InteropServices;

public class Drone : MonoBehaviour
{
    //  ┌--------------------┐
    //  │   Basic Paramter   │
    //  └--------------------┘
    public double dt = 0.01;
    public double init_Pos_N = 0;
    public double init_Pos_E = 0;
    public double init_Pos_D = 0;
    public double init_Att_Roll = 0;
    public double init_Att_Pitch = 0;
    public double init_Att_Yaw = 0;
    double g = 9.81;
    double D2R = Math.PI / 180;
    double R2D = 180 / Math.PI;
    double R;
    double M = 6378137.0000;
    double N = 6356752.3142;
    double[] VecT;
    double[,] Gvec = new double[3, 1];
    double[,] sigma = new double[3, 1];
    double sigma_norm, a_c, a_s;


    //  ┌--------------------┐
    //  │  Filter Variables  │
    //  └--------------------┘
    public double[,] F_Pos = new double[3, 1];
    public double[,] F_Vel = new double[3, 1];


    //  ┌--------------------┐
    //  │       Drone        │
    //  └--------------------┘
    double[,] Pre_B_dW = new double[3, 1];
    double[,] B_dW = new double[3, 1];
    double[,] B_W = new double[3, 1];
    double[,] T_Acc = new double[3, 1];
    double[,] T_bAcc = new double[3, 1];
    public double[,] T_Euler = new double[3, 1];
    public double[,] F_Euler = new double[3, 1];
    double[,] T_Cbn = new double[3, 3];
    double[,] T_Quat = new double[4, 1];
    double[,] T_Quat_Update = new double[4, 1];
    double[,] Pre_T_Acc = new double[3, 1];
    double[,] Pre_T_Vel = new double[3, 1];
    public double[,] T_Vel = new double[3, 1];
    public double[,] T_Pos = new double[3, 1];
    double Thrust, m, L, Ixx, Iyy, Izz, Irr;
    double[,] W = new double[4, 1];
    public double[,] D_W = new double[4, 1];
    public double df = 0.98;
 
    //  ┌--------------------┐
    //  │      Control       │
    //  └--------------------┘
    double[,] C_Att = new double[3, 1];
    double[,] Target_Att = new double[3, 1];

    double[,] CMD_Pos = new double[3, 1];
    double[,] CMD_Att = new double[3, 1];

    double[,] E_Att = new double[3, 1];
    double[,] Pre_E_Att = new double[3, 1];
    double[,] inte_E_Att = new double[3, 1];
    double[,] E_Pos = new double[3, 1];
    double[,] E_Vel = new double[3, 1];
    double[,] Pre_E_Vel = new double[3, 1];
    double[,] inte_E_Vel = new double[3, 1];
    double[,] C_Vel = new double[3, 1];
    double[,] Target_Vel = new double[3, 1];
    public double Max_Att = 20;
    public double Cut_Pos = 20;
    public double Max_Vel = 5;
    public double A_Kp = 5;
    public double A_Ki = 0;
    public double A_Kd = 5;
    public double L_Kp = 5;
    public double L_Ki = 0;
    public double L_Kd = 10;
    public double Max_W = 100;

    private List<Matrix4x4> markerTransforms = new List<Matrix4x4>();
    public RenderTexture IMG; 
    public bool Flag = false;

    FileStream F;
    StreamWriter SW;
    string S_DH;
    string S_PN,S_PE,S_PD,S_Pos;
    string S_VN,S_VE,S_VD,S_Vel;
    string S_BAN,S_BAE,S_BAD,S_bAcc;
    string S_Roll,S_Pitch,S_Yaw,S_Att;
    string S_GX,S_GY,S_GZ,S_Gyro;
    string TAP = "\t";
    string Data_Set;
    int imgCount =0;
    int Data_Header=0;
    string PATH = "Assets/Log/";
    string IMGPATH = "Assets/Log/IMG/";

    //  ┌--------------------┐
    //  │   Visualization    │
    //  └--------------------┘
    Vector3 V_Att = new Vector3();
    Vector3 V_rAtt = new Vector3();
    Vector3 V_Pos = new Vector3();

    int Type_Flag = 1;
    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = (int)(1 / dt);
        init_Dynamics();
    }

    // Update is called once per frame
    void Update()
    {
        Drone_Dynamics();
        T_Filter();
        //RC_Input_A();
        RC_Input_P();
        //Drone_Att_Control();
        Drone_Pos_Control();
        Visualization();
        
        if(Flag)
        {
            Save_Data();
        }
        //Debug.Log("T Pos : " + (T_Pos[0, 0]).ToString() + "\t" + (T_Pos[1, 0]).ToString() + "\t" + (T_Pos[2, 0]).ToString());
        //Debug.Log("T Vel : " + (T_Vel[0, 0]).ToString() + "\t" + (T_Vel[1, 0]).ToString() + "\t" + (T_Vel[2, 0]).ToString());
        //Debug.Log("CMD : " + CMD_Att[0, 0].ToString() + "\t" + CMD_Att[1, 0].ToString() + "\t" + CMD_Att[2, 0].ToString());
        //Debug.Log("T Att : " + (T_Euler[0, 0] * R2D).ToString() + "\t" + (T_Euler[1, 0] * R2D).ToString() + "\t" + (T_Euler[2, 0] * R2D).ToString());
        //Debug.Log("F Att : " + (F_Euler[0, 0] * R2D).ToString() + "\t" + (F_Euler[1, 0] * R2D).ToString() + "\t" + (F_Euler[2, 0] * R2D).ToString());
        //Debug.Log("Rotor : " + D_W[0, 0].ToString() + "\t" + D_W[1, 0].ToString() + "\t" + D_W[2, 0].ToString() + "\t" + D_W[3, 0].ToString());
    }

    public void Save_Data()
    {
        imgCount++;
        Data_Header++;
        S_DH = Data_Header.ToString();
        S_PN = T_Pos[0,0].ToString("F10");
        S_PE = T_Pos[1,0].ToString("F10");
        S_PD = T_Pos[2,0].ToString("F10");
        S_Pos = S_PN+TAP+S_PE+TAP+S_PD;
        S_VN = T_Vel[0,0].ToString("F10");
        S_VE = T_Vel[1,0].ToString("F10");
        S_VD = T_Vel[2,0].ToString("F10");
        S_Vel = S_VN+TAP+S_VE+TAP+S_VD;
        S_BAN = T_bAcc[0,0].ToString("F10");
        S_BAE = T_bAcc[1,0].ToString("F10");
        S_BAD = T_bAcc[2,0].ToString("F10");
        S_bAcc = S_BAN+TAP+S_BAE+TAP+S_BAD;
        S_Roll = T_Euler[0,0].ToString("F10");
        S_Pitch = T_Euler[1,0].ToString("F10");
        S_Yaw = T_Euler[2,0].ToString("F10");
        S_Att = S_Roll+TAP+S_Pitch+TAP+S_Yaw;
        S_GX = B_W[0,0].ToString("F10");
        S_GY = B_W[1,0].ToString("F10");
        S_GZ = B_W[2,0].ToString("F10");
        S_Gyro=S_GX+TAP+S_GY+TAP+S_GZ;
        
        Data_Set = S_DH+TAP+S_Pos+TAP+S_Vel+TAP+S_Att+TAP+S_bAcc+TAP+S_Gyro;
        SW.WriteLine(Data_Set);
        if(imgCount>=10)
        {
            imgCount=0;
            byte[] F_IMG = toPNG(IMG);
            File.WriteAllBytes(IMGPATH+S_DH+".png",F_IMG);
        }
        
    }
    byte[] toPNG(RenderTexture rTex)
    {
        Texture2D tex = new Texture2D(256, 256, TextureFormat.RGB24, false);
        RenderTexture.active = rTex;
        tex.ReadPixels(new UnityEngine.Rect(0, 0, rTex.width, rTex.height), 0, 0);
        tex.Apply();
        byte[] Output = tex.EncodeToPNG();
        return Output;
    }
    public void Save_Button()
    {
        Flag = true;
        F = new FileStream( PATH  + "Data.txt", FileMode.Append, FileAccess.Write);
        SW = new StreamWriter(F, System.Text.Encoding.Unicode);
        Data_Header = 0;
        imgCount = 0;
    }
    public void Stop_Buton()
    {
        Flag = false;
        F.Close();
    }
    //  ┌--------------------┐
    //  │    True  Filter    │
    //  └--------------------┘
    public void T_Filter()
    {
        F_Euler = T_Euler;
        F_Pos = T_Pos;
        F_Vel = T_Vel;
    }

    //  ┌--------------------┐
    //  │   Drone Dynamics   │
    //  └--------------------┘
    public void init_Dynamics()
    {
        E_Att = Make_Vec(VecT = new double[3] { 0, 0, 0 });
         
        T_Pos = Make_Vec(VecT = new double[3] { init_Pos_N, init_Pos_E, init_Pos_D });
        T_Vel = Make_Vec(VecT = new double[3] { 0, 0, 0 });
        T_Acc = Make_Vec(VecT = new double[3] { 0, 0, 0 });
        
        T_Euler = Make_Vec(VecT = new double[3] { init_Att_Roll, init_Att_Pitch, init_Att_Yaw });
        T_Cbn = Cal_Euler_2_Cbn(T_Euler);
        T_Quat = Cal_Cbn_2_Quat(T_Cbn);

        Ixx = 0.006;
        Iyy = 0.006;
        Izz = 0.011; //kg/m^2
        Irr = 0.026; // kg/m^2/%
        L = 0.2;
        m = 0.727;

        double C_Thrust = m * g / Irr / (Math.Cos(T_Euler[0, 0]) * Math.Cos(T_Euler[1, 0]));
        W[0, 0] = C_Thrust / 4;
        W[1, 0] = C_Thrust / 4;
        W[2, 0] = C_Thrust / 4;
        W[3, 0] = C_Thrust / 4;
    }
    public void Drone_Dynamics()
    {
        double[,] F_Vec;
        Pre_B_dW = B_dW;
        //중력 계산
        R = Math.Sqrt(M * N);
        ///R = R0;
        Gvec[2, 0] = g;

        //D_W = Mat_Cal(Mat_Scalar(alpha, D_W), '+', Mat_Scalar(1 - alpha, W));
        D_W = W;
        // Attitude
        Thrust = Irr / m * (D_W[0, 0] + D_W[1, 0] + D_W[2, 0] + D_W[3, 0]);
        B_dW[0, 0] = L * Irr / Ixx * (D_W[0, 0] - D_W[1, 0] - D_W[2, 0] + D_W[3, 0]);
        B_dW[1, 0] = L * Irr / Iyy * (D_W[0, 0] + D_W[1, 0] - D_W[2, 0] - D_W[3, 0]);
        B_dW[2, 0] = L * Irr / Izz * (D_W[0, 0] - D_W[1, 0] + D_W[2, 0] - D_W[3, 0]);
        B_W = Vec_integate(B_W, Pre_B_dW, B_dW, dt);

        sigma = Mat_Scalar(dt, B_W);
        sigma_norm = Vec_Norm(sigma);
        a_c = Math.Cos(sigma_norm / 2);
        if (sigma_norm == 0)
        {
            a_s = 0.5;
        }
        else
        {
            a_s = Math.Sin(sigma_norm / 2) / sigma_norm;
        }
        T_Quat_Update = Make_Vec(VecT = new double[4] { a_c, a_s * sigma[0, 0], a_s * sigma[1, 0], a_s * sigma[2, 0] });
        T_Quat = Cal_Quat_Multi(T_Quat, T_Quat_Update);
        T_Quat = Mat_Scalar(1 / Vec_Norm(T_Quat), T_Quat);
        T_Cbn = Cal_Quat_2_Cbn(T_Quat);
        T_Euler = Cal_Cbn_2_Euler(T_Cbn);

        Pre_T_Acc = T_Acc;
        Pre_T_Vel = T_Vel;
        // Position
        F_Vec = Make_Vec(VecT = new double[3]{0, 0, -Thrust});
        T_Acc = Mat_Multi(T_Cbn,F_Vec);
        T_Acc = Mat_Cal(T_Acc,'+',Gvec);
        T_Acc = Mat_Cal(T_Acc, '-',Mat_Scalar(df, T_Vel));
        T_bAcc = Mat_Multi(Mat_Transpose(T_Cbn), Mat_Cal(T_Acc, '-', Gvec));
        T_Vel = Vec_integate(T_Vel, Pre_T_Acc, T_Acc, dt);
        T_Pos = Vec_integate(T_Pos, Pre_T_Vel, T_Vel, dt);
    }
    public void Drone_Att_Control()
    {
        double C_Thrust;
        Target_Att = Mat_Scalar(D2R, CMD_Att);
        Target_Att[2, 0] = 0;

        Pre_E_Att = E_Att;
        E_Att = Mat_Cal(Target_Att, '-', F_Euler);
        inte_E_Att = Mat_Cal(inte_E_Att, '+', Mat_Scalar(dt, E_Att));

        for (int i = 0; i < 3; i++)
        {
            C_Att[i, 0] = A_Kp * E_Att[i, 0] + A_Ki * inte_E_Att[i, 0] + A_Kd * (E_Att[i, 0] - Pre_E_Att[i, 0]) / dt;
        }
        C_Thrust = m * g / Irr / (Math.Cos(F_Euler[0, 0]) * Math.Cos(F_Euler[1, 0])) - CMD_Pos[2, 0];

        W[0, 0] = C_Thrust / 4 + C_Att[0, 0] / 4 + C_Att[1, 0] / 4 + C_Att[2, 0] / 4;
        W[1, 0] = C_Thrust / 4 - C_Att[0, 0] / 4 + C_Att[1, 0] / 4 - C_Att[2, 0] / 4;
        W[2, 0] = C_Thrust / 4 - C_Att[0, 0] / 4 - C_Att[1, 0] / 4 + C_Att[2, 0] / 4;
        W[3, 0] = C_Thrust / 4 + C_Att[0, 0] / 4 - C_Att[1, 0] / 4 - C_Att[2, 0] / 4;
        for(int i=0; i<4; i++)
        {
            if (W[i, 0] > Max_W)
                W[i, 0] = Max_W;
            else if (W[i, 0] < 0)
                W[i, 0] = 0;
        }
    }
    public void Drone_Pos_Control()
    {
        double C_Thrust;

        E_Pos = Mat_Cal(CMD_Pos, '-', F_Pos);

        for (int i = 0; i < 3; i++)
        {
            if (E_Pos[i, 0] > Cut_Pos)
                Target_Vel[i, 0] = Max_Vel;
            else if (E_Pos[i, 0] < -Cut_Pos)
                Target_Vel[i, 0] = -Max_Vel;
            else
                Target_Vel[i, 0] = E_Pos[i, 0];
        }

        Pre_E_Vel = E_Vel;
        E_Vel = Mat_Cal(Target_Vel, '-', F_Vel);
        inte_E_Vel = Mat_Cal(inte_E_Vel, '+', Mat_Scalar(dt, E_Vel));

        for (int i = 0; i < 3; i++)
        {
            C_Vel[i, 0] = L_Kp * E_Vel[i, 0] + L_Ki * inte_E_Vel[i, 0] + L_Kd * (E_Vel[i, 0] - Pre_E_Vel[i, 0]) / dt;
        }

        for (int i = 0; i < 2; i++)
        {
            if (C_Vel[i, 0] > Max_Att)
                C_Vel[i, 0] = Max_Att;
            else if (C_Vel[i, 0] < -Max_Att)
                C_Vel[i, 0] = -Max_Att;
            else
                C_Vel[i, 0] = C_Vel[i, 0];
        }
        Target_Att[0, 0] =  -C_Vel[1, 0];
        Target_Att[1, 0] =  -C_Vel[0, 0];
        Target_Att[2, 0] = 0;

        Pre_E_Att = E_Att;
        E_Att = Mat_Cal(Mat_Scalar(D2R,Target_Att), '-', F_Euler);
        inte_E_Att = Mat_Cal(inte_E_Att, '+', Mat_Scalar(dt, E_Att));

        for (int i = 0; i < 3; i++)
        {
            C_Att[i, 0] = A_Kp * E_Att[i, 0] + A_Ki * inte_E_Att[i, 0] + A_Kd * (E_Att[i, 0] - Pre_E_Att[i, 0]) / dt;
        }
        C_Thrust = m * g / Irr / Math.Cos(F_Euler[0, 0]) * Math.Cos(F_Euler[1, 0]) - C_Vel[2, 0];

        W[0, 0] = C_Thrust / 4 + C_Att[0, 0] / 4 + C_Att[1, 0] / 4 + C_Att[2, 0] / 4;
        W[1, 0] = C_Thrust / 4 - C_Att[0, 0] / 4 + C_Att[1, 0] / 4 - C_Att[2, 0] / 4;
        W[2, 0] = C_Thrust / 4 - C_Att[0, 0] / 4 - C_Att[1, 0] / 4 + C_Att[2, 0] / 4;
        W[3, 0] = C_Thrust / 4 + C_Att[0, 0] / 4 - C_Att[1, 0] / 4 - C_Att[2, 0] / 4;
        for (int i = 0; i < 4; i++)
        {
            if (W[i, 0] > Max_W)
                W[i, 0] = Max_W;
            else if (W[i, 0] < 0)
                W[i, 0] = 0;
        }
    }
    
    //  ┌--------------------┐
    //  │  Vector Calculate  │
    //  └--------------------┘
    public double[,] Make_Vec(double[] List)
    {  
        int n = List.GetLength(0);
        double[,] result = new double[n, 1];
        for (int i = 0; i < n; i++)
        {
            result[i, 0] = List[i];
        }
        return result;
    }
    public double Vec_Norm(double[,] Vec)
    {
        int n = Vec.GetLength(0);

        double result = 0.0;

        for (int i = 0; i < n; i++)
        {
            result += Vec[i, 0] * Vec[i, 0];
        }
        result = Math.Sqrt(result);

        return result;
    }
    public double[,] Vec_integate(double[,] Pre_Vec, double[,] Pre_dVec, double[,] dVec, double dt)
    {
        int n = Pre_Vec.GetLength(0);
        double[,] result = new double[n, 1];

        for (int i = 0; i < n; i++)
        {
            result[i, 0] = Pre_Vec[i, 0] + (0.5 * (Pre_dVec[i, 0] + dVec[i, 0]) * dt);
        }

        return result;
    }
  

    //  ┌--------------------┐
    //  │  Matrix Calculate  │
    //  └--------------------┘
    public double[,] Mat_Multi(double[,] m1, double[,] m2)
    {
        double[,] result = new double[m1.GetLength(0), m2.GetLength(1)];

        if (m1.GetLength(1) == m2.GetLength(0))
        {
            for (int i = 0; i < result.GetLength(0); i++)
            {
                for (int j = 0; j < result.GetLength(1); j++)
                {
                    result[i, j] = 0;
                    for (int k = 0; k < m1.GetLength(1); k++)
                        result[i, j] = result[i, j] + m1[i, k] * m2[k, j];
                }
            }
        }
        return result;
    }
    public double[,] Mat_Transpose(double[,] Mat)
    {
        int m = Mat.GetLength(0);
        int n = Mat.GetLength(1);
        double[,] result = new double[n, m];

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                result[j, i] = Mat[i, j];
            }
        }

        return result;
    }
    public double[,] Mat_Scalar(double K, double[,] Mat)
    {
        int m = Mat.GetLength(0);
        int n = Mat.GetLength(1);
        double[,] result = new double[m, n];

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                result[i, j] = K * Mat[i, j];
            }
        }

        return result;
    }
    public double[,] Mat_Cal(double[,] Mat1, char Cal, double[,] Mat2)
    {
        int m = Mat1.GetLength(0);
        int n = Mat1.GetLength(1);
        double[,] result = new double[m, n];

        if (Cal == '+')
        {

            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = Mat1[i, j] + Mat2[i, j];
                }
            }
        }
        else if (Cal == '-')
        {
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = Mat1[i, j] - Mat2[i, j];
                }
            }
        }
        return result;
    }


    //  ┌--------------------┐
    //  │   Nav Calculate    │
    //  └--------------------┘
    public double[,] Cal_Euler_2_Cbn(double[,] In)
    {
        double[,] Out = new double[3, 3];
        double R, P, Y;

        R = In[0, 0];
        P = In[1, 0];
        Y = In[2, 0];

        Out[0, 0] = (Math.Cos(P) * Math.Cos(Y));
        Out[0, 1] = (-Math.Cos(R) * Math.Sin(Y)) + (Math.Sin(R) * Math.Sin(P) * Math.Cos(Y));
        Out[0, 2] = (Math.Sin(R) * Math.Sin(Y)) + (Math.Cos(R) * Math.Sin(P) * Math.Cos(Y));

        Out[1, 0] = (Math.Cos(P) * Math.Sin(Y));
        Out[1, 1] = (Math.Cos(R) * Math.Cos(Y)) + (Math.Sin(R) * Math.Sin(P) * Math.Sin(Y));
        Out[1, 2] = (-Math.Sin(R) * Math.Cos(Y)) + (Math.Cos(R) * Math.Sin(P) * Math.Sin(Y));

        Out[2, 0] = (-Math.Sin(P));
        Out[2, 1] = (Math.Sin(R) * Math.Cos(P));
        Out[2, 2] = (Math.Cos(R) * Math.Cos(P));


        return Out;
    }
    public double[,] Cal_Cbn_2_Euler(double[,] In)
    {
        double[,] Out = new double[3, 1];

        Out[0, 0] = Math.Atan(In[2, 1] / In[2, 2]);
        Out[1, 0] = Math.Asin(-In[2, 0]);
        Out[2, 0] = Math.Atan2(In[1, 0], In[0, 0]);

        return Out;
    }
    public double[,] Cal_Quat_2_Cbn(double[,] In)
    {
        double[,] Out = new double[3, 3];
        double a, b, c, d;
        a = In[0, 0];
        b = In[1, 0];
        c = In[2, 0];
        d = In[3, 0];

        Out[0, 0] = (a * a) + (b * b) - (c * c) - (d * d);
        Out[0, 1] = 2 * ((b * c) - (a * d));
        Out[0, 2] = 2 * ((b * d) + (a * c));

        Out[1, 0] = 2 * ((b * c) + (a * d));
        Out[1, 1] = (a * a) - (b * b) + (c * c) - (d * d);
        Out[1, 2] = 2 * ((c * d) + (a * b));

        Out[2, 0] = 2 * ((b * d) - (a * c));
        Out[2, 1] = 2 * ((c * d) + (a * b));
        Out[2, 2] = (a * a) - (b * b) - (c * c) + (d * d);

        return Out;
    }
    public double[,] Cal_Cbn_2_Quat(double[,] In)
    {
        double[,] Out = new double[4, 1];
        double a, b, c, d;
        a = 0.5 * Math.Sqrt(1 + In[0, 0] + In[1, 1] + In[2, 2]);
        b = (1 / (4 * a)) * (In[2, 1] - In[1, 2]);
        c = (1 / (4 * a)) * (In[0, 2] - In[2, 0]);
        d = (1 / (4 * a)) * (In[1, 0] - In[0, 1]);

        Out[0, 0] = a;
        Out[1, 0] = b;
        Out[2, 0] = c;
        Out[3, 0] = d;

        return Out;
    }
    public double[,] Cal_Quat_Multi(double[,] Q1, double[,] Q2)
    {
        double[,] Out = new double[4, 1];
        double[,] Temp = new double[4, 4];

        Temp[0, 0] = Q1[0, 0];
        Temp[1, 0] = Q1[1, 0];
        Temp[2, 0] = Q1[2, 0];
        Temp[3, 0] = Q1[3, 0];

        Temp[0, 1] = -Q1[1, 0];
        Temp[1, 1] = Q1[0, 0];
        Temp[2, 1] = Q1[3, 0];
        Temp[3, 1] = -Q1[2, 0];

        Temp[0, 2] = -Q1[2, 0];
        Temp[1, 2] = -Q1[3, 0];
        Temp[2, 2] = Q1[0, 0];
        Temp[3, 2] = Q1[1, 0];

        Temp[0, 3] = -Q1[3, 0];
        Temp[1, 3] = Q1[2, 0];
        Temp[2, 3] = -Q1[1, 0];
        Temp[3, 3] = Q1[0, 0];

        Out = Mat_Multi(Temp, Q2);
        return Out;
    }

    //  ┌--------------------┐
    //  │  Vector Calculate  │
    //  └--------------------┘
    public void Visualization()
    {
        V_Pos = new Vector3((float)(F_Pos[1, 0]), -(float)(F_Pos[2, 0]), -(float)(F_Pos[0, 0]));
        transform.position = V_Pos;
        V_Att = new Vector3((float)(R2D * F_Euler[1, 0]), (float)(R2D * F_Euler[2, 0]), (float)(R2D * T_Euler[0, 0]));
        V_rAtt = transform.rotation.eulerAngles;
        transform.Rotate(V_Att - V_rAtt);
    }

    //  ┌--------------------┐
    //  │   Command  Input   │
    //  └--------------------┘
    public void RC_Input_A()
    {
        if (Input.GetKey(KeyCode.W))
        {
            CMD_Att[1, 0] = 20;
        }
        else if (Input.GetKey(KeyCode.X))
        {
            CMD_Att[1, 0] = -20;
        }
        else if (Input.GetKey(KeyCode.A))
        {
            CMD_Att[0, 0] = 20;
        }
        else if (Input.GetKey(KeyCode.D))
        {
            CMD_Att[0, 0] = -20;
        }
        else if (Input.GetKey(KeyCode.Q))
        {
            CMD_Att[2, 0] -= 1;
        }
        else if (Input.GetKey(KeyCode.E))
        {
            CMD_Att[2, 0] += 1;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            CMD_Att[0, 0] = 0;
            CMD_Att[1, 0] = 0;
            CMD_Att[2, 0] = 0;
        }
        else if (Input.GetKey(KeyCode.UpArrow))
        {
           CMD_Pos[2, 0] = -20;
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            CMD_Pos[2, 0] = 20;
        }
        else if (Input.GetKey(KeyCode.Space))
        {
            CMD_Pos[2, 0] = 0;
        }
    }
    public void RC_Input_P()
    {
    if (Input.GetKey(KeyCode.Q))
        {
            CMD_Pos[0, 0] = 0;
            CMD_Pos[1, 0] = 0;
            CMD_Pos[2, 0] = 0;
        }
        else if (Input.GetKey(KeyCode.W))
        {
            CMD_Pos[0, 0] = 0;
            CMD_Pos[1, 0] = 0;
            CMD_Pos[2, 0] = -7;
        }
        else if (Input.GetKey(KeyCode.E))
        {
            CMD_Pos[0, 0] = -25;
            CMD_Pos[1, 0] = 0;
            CMD_Pos[2, 0] = -7;
        }
        else if (Input.GetKey(KeyCode.A))
        {
            CMD_Pos[0, 0] = -27;
            CMD_Pos[1, 0] = 3;
            CMD_Pos[2, 0] = -7;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            CMD_Pos[0, 0] = -22;
            CMD_Pos[1, 0] = -2;
            CMD_Pos[2, 0] = -7;
        }
    }
  
}

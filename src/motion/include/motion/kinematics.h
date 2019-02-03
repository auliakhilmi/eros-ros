#ifndef KINEMATICS_H
#define KINEMATICS_H
#define id_max 20

class Kinematics{
    private:
        float Angle_Kaki[6] = {0,0,0,0,0,0};
        float Angle_Tangan[8] = {0,0,0,0,0,0,0,0};
        float Angle_Kepala[3] = {0,0,0};
        float Angle_Waist[3] = {0,0,0};
    public:
        float AngleJointAll[id_max+1];
        void InversKinematic(float x, float y, float z, int Heading);
        void CoordinateKaki(float x1, float y1, float z1, int Heading1, float x2, float y2, float z2, int Heading2, float AngleBody);
};

void Kinematics::InversKinematic(float x, float y, float z, int Heading){
  float Resultan0, Resultan1, Resultan2, aX, aZ, Alfa;
  float sA = 0, sB = 0, sC = 0;
  float sBA, sBB, sCA, sCB, sD;

  Angle_Kaki[0] = Heading;//servo no 7-8
  Resultan0=0;
  Alfa=0;
  //Resultan0 = sqrt((y*y) + (x*x));

  //Alfa =  (atan2(y,x) * PHI) - Heading;
  aZ = z + (sin(Alfa / PHI) * Resultan0);
  aX = x - (cos(Alfa / PHI) * Resultan0);

  Angle_Kaki[1] = atan2(aZ,y) * PHI;

  Resultan1 = sqrt(aZ*aZ + y*y);
  if(Resultan1 >= Frame3)Resultan1 = Frame3;
  Resultan2 = sqrt((Resultan1*Resultan1) + (aX*aX));
  if(Resultan2 >= Frame3)Resultan2 = Frame3;

  sCA = (float)((Frame1*Frame1) + (Frame2*Frame2)) - (Resultan2*Resultan2);
  sCB = (float)(2 * Frame1 * Frame2);
  sC  = (float)acos(sCA / sCB) * PHI ;
  //sA = (float)atan2(Resultan1,-x) * PHI;
  sA  = asin(x/Resultan2)*PHI;
  //sBA = (float)Frame2 * sin(sC / PHI);
  //sBB = (float)Frame1 + (Frame2 * cos(sC / PHI));
  //sB  = (float)atan2(sBA,sBB) * PHI;
  sB  = asin((Frame2 * sin(sC / PHI))/Resultan2) * PHI;

  Angle_Kaki[2] = 0 - (sA + sB);
  Angle_Kaki[3] = (180 - sC);
  Angle_Kaki[4] = (Angle_Kaki[2] + Angle_Kaki[3]);
  Angle_Kaki[5] = -Angle_Kaki[1]; //----- 
}

void Kinematics::CoordinateKaki(float x1, float y1, float z1, int Heading1, float x2, float y2, float z2, int Heading2, float AngleBody){
    int dataMiring;
    if(state != 20)dataMiring = 0; //err_y with IMU
    else dataMiring = 0;

    //--------------------Kaki Kanan--------------------------------------------------------
    Kinematics::InversKinematic(x1, y1, z1, Heading1);
    AngleJointAll[7] = Angle_Kaki[0];
    if(y1 > y2 && Angle_Kaki[1] >= 0){
      AngleJointAll[9] = (Angle_Kaki[1] - AngleBody * sin(Angle_Kaki[0] / PHI))* 0.3 + (float)dataMiring / 6;
    }else{
      AngleJointAll[9] = (Angle_Kaki[1] - AngleBody * sin(Angle_Kaki[0] / PHI)) + (float)dataMiring / 6;
    }
    //ROS_INFO("AK0[%1.2f]AK1[%1.2f]AB[%1.2f]9[%1.2f]",Angle_Kaki[0],Angle_Kaki[1],AngleBody,AngleJointAll[9]);
    AngleJointAll[11] = Angle_Kaki[2] - AngleBody * cos(Angle_Kaki[0] / PHI);
    AngleJointAll[13] = Angle_Kaki[3];
    if(y1 > y2){
      AngleJointAll[15] = Angle_Kaki[4] + (y1 - y2)/15;
    }else{
      AngleJointAll[15] = Angle_Kaki[4];
    }

    AngleJointAll[17] = (Angle_Kaki[5]) - (float)dataMiring/6;


    //--------------------Kaki Kiri---------------------------------------------------------
    Kinematics::InversKinematic(x2, y2, z2, Heading2);
    AngleJointAll[8] = Angle_Kaki[0];
    if(y1 < y2 && Angle_Kaki[1] <= 0){
      AngleJointAll[10] = (Angle_Kaki[1] - AngleBody * sin(Angle_Kaki[0] / PHI))* 0.3 + (float)dataMiring / 6;
    }else{
      AngleJointAll[10] = Angle_Kaki[1]  - AngleBody * sin(Angle_Kaki[0] / PHI) + (float)dataMiring / 6;
    }
    AngleJointAll[12] = Angle_Kaki[2] - AngleBody * cos(Angle_Kaki[0] / PHI);//- AngleBody * cos(Angle[0] / PHI);
    AngleJointAll[14] = Angle_Kaki[3];
    if(y2 > y1){
      AngleJointAll[16] = (Angle_Kaki[4] + (y2 - y1)/15);
    }else{
      AngleJointAll[16] = Angle_Kaki[4];
    }
    AngleJointAll[18] = Angle_Kaki[5] - (float)dataMiring/6;

    //------------------Tangan--------------------------------------------------------------
    AngleJointAll[1] = -13.17;
    AngleJointAll[2] = 13.17;
    AngleJointAll[3] = -60;
    AngleJointAll[4] = 60;
    AngleJointAll[5] = -60.7;
    AngleJointAll[6] = 60.7;
  }

#endif // KINEMATICS_H

#ifndef MOTION_H
#define MOTION_H

#include <std_msgs/UInt32.h>
#include <geometry_msgs/Quaternion.h>
#include "motion/loadparameter.h"
#include "motion/definition.h"
#include "motion/globalvars.h"
#include "motion/kinematics.h"
class Motion4step{
    private :
      Parameter parameter;
      Kinematics kinematics;
      ros::NodeHandle node[4];
      ros::Publisher pub[4];
      unsigned char step=1;
      float aBody, t;        
      //float xFoot1, yFoot1, zFoot1, xFoot2, yFoot2, zFoot2, hFoot1, hFoot2;
      int motions;
      int t_action[11];
      int PatternMotion[8][8] = {
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
      };
      short int DefaultStepTrajectoryWalking[8][8] = {
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
          {0, 200, 0, 0, 0, 200, 0, 0},
      };
      int sMotion[18][11];
      float AnglePosition[34];
      int max_step;
      int cKick;

      void CalculateValue();

    public :
        Motion4step();
        void ChangeDataMotion(int fMth);
        void MotionCounter(int fMth);
        void StepTrajectoryWalking(int fMth);
        void BasicMotionFSM();
        void ChangeDataAction(int fMth);
        void StepTrajectoryAction(int fMth);
        void motion_publish();
        void InertialFeedback();
        void TrajectoryShooting(int fMth);
        void StepTrajectoryKicking();
};

Motion4step::Motion4step()
{
    Motion4step::pub[0] = Motion4step::node[0].advertise<geometry_msgs::Quaternion>("/motion/koordinat/kaki_kanan",1);
    Motion4step::pub[1] = Motion4step::node[1].advertise<geometry_msgs::Quaternion>("/motion/koordinat/kaki_kiri",1);
    Motion4step::pub[2] = Motion4step::node[2].advertise<std_msgs::UInt32>("/motion/step",1);
    Motion4step::pub[3] = Motion4step::node[3].advertise<std_msgs::UInt32>("/motion/motion",1);

    parameter.servoPosition(DefaultServo,0);
    parameter.servoPosition(DS_Origin,0);
    parameter.servoPosition(DS_Backup,0);
    parameter.servoPosition(GoalPosition,1);

    t=10;
}

void Motion4step::motion_publish()
{
    geometry_msgs::Quaternion koordinat_kaki_kanan_p;
    geometry_msgs::Quaternion koordinat_kaki_kiri_p;
    std_msgs::UInt32 step_p;
    std_msgs::UInt32 motion_p;

    koordinat_kaki_kanan_p.x = zFoot1;
    koordinat_kaki_kanan_p.y = xFoot1;
    koordinat_kaki_kanan_p.z = yFoot1;
    koordinat_kaki_kanan_p.w = hFoot1;

    koordinat_kaki_kiri_p.x = zFoot2;
    koordinat_kaki_kiri_p.y = xFoot2;
    koordinat_kaki_kiri_p.z = yFoot2;
    koordinat_kaki_kiri_p.w = hFoot2;

    step_p.data = step;
    motion_p.data = motions;

    Motion4step::pub[0].publish(koordinat_kaki_kanan_p);
    Motion4step::pub[1].publish(koordinat_kaki_kiri_p);
    Motion4step::pub[2].publish(step_p);
    Motion4step::pub[3].publish(motion_p);
}

void Motion4step::ChangeDataMotion(int fMth){
    int sWalk[5][8];
    parameter.getparam(fMth,sWalk,t,aBody);
    for(j=0 ; j<8 ; j++){
        for(i=0 ; i<5 ; i++){
            PatternMotion[i][j] = sWalk[i][j];
        }
    }
}

//MOTION COUNTER
void Motion4step::MotionCounter(int fMth){
  //________________________________Acceleration___________________________//
  if(fMth==50){
    if(counter<4)fMth=10;
    else if(counter>=4 && counter<8)fMth=20;
    else if(counter>=8 && counter<12)fMth=30;
    else if(counter>=12 && counter<16)fMth=40;
  }
  else if(fMth==51){
    if(counter<4)fMth=11;
    else if(counter>=4 && counter<8)fMth=21;
    else if(counter>=8 && counter<12)fMth=31;
    else if(counter>=12 && counter<16)fMth=41;
  }
  else if(fMth==52){
    if(counter<4)fMth=12;
    else if(counter>=4 && counter<8)fMth=22;
    else if(counter>=8 && counter<12)fMth=32;
    else if(counter>=12 && counter<16)fMth=42;
  }
  else if(fMth==40){
    if(counter<4)fMth=10;
    else if(counter>=4 && counter<8)fMth=20;
    else if(counter>=8 && counter<12)fMth=30;
  }
  else if(fMth==41){
    if(counter<4)fMth=11;
    else if(counter>=4 && counter<8)fMth=21;
    else if(counter>=8 && counter<12)fMth=31;
  }
  else if(fMth==42){
    if(counter<4)fMth=12;
    else if(counter>=4 && counter<8)fMth=22;
    else if(counter>=8 && counter<12)fMth=32;
  }
  else if(fMth>=30 && fMth<40){
    if(counter<4)fMth=10;
    else if(counter>=4 && counter<8)fMth=20;
  }
  else if(fMth>=20 && fMth<30){
    if(counter<4)fMth=10;
  }
  //________________________________DeAcceleration___________________________//
  if(fMth>=30 && fMth<40){
    if(dcount<4)fMth=40;
  }
  else if(fMth>=20 && fMth<30){
      if(dcount<4)fMth=40;
      else if(dcount>=4 && dcount<8)fMth=30;
  }
  else if(fMth>200 || (fMth>=10 && fMth<19)){
      if(fMth == 11){
          if(dcount<4)fMth=41;
          else if(dcount>=4 && dcount<8)fMth=31;
          else if(dcount>=8 && dcount<12)fMth=21;
      }else if(fMth == 12){
          if(dcount<4)fMth=42;
          else if(dcount>=4 && dcount<8)fMth=32;
          else if(dcount>=8 && dcount<12)fMth=22;
      }else{
          if(dcount<4)fMth=40;
          else if(dcount>=4 && dcount<8)fMth=30;
          else if(dcount>=8 && dcount<12)fMth=20;
      }
  }
  else if(fMth == 19 || fMth < 10){
    if(dcount<4)fMth=40;
    else if(dcount>=4 && dcount<8)fMth=30;
    else if(dcount>=8 && dcount<12)fMth=20;
    else if(dcount>=12 && dcount<16)fMth=10;
  }

  if(fMth!=lastfMth){
    ChangeDataMotion(fMth);
    lastfMth=fMth;
  }

  if(fMth==19 || fMth<=9){counter=0;if(dcount>=18)dcount=18;}
  else if(fMth>=10 && fMth<19){if(counter>=4)counter=4;if(dcount>=16)dcount=16;}
  else if(fMth>=20 && fMth<25){if(counter>=8)counter=8;if(dcount>=12)dcount=12;}
  else if(fMth>=30 && fMth<40){if(counter>=12)counter=12;if(dcount>=8)dcount=8;}
  else if(fMth>=40 && fMth<50){if(counter>=16)counter=16;if(dcount>=4)dcount=4;}
  else if(fMth>=50 && fMth<60){if(counter>=18)counter=18; dcount=0;}

  if(counter >= 18) counter = 18;
  if(dcount >= 18)dcount = 18;
  counter++;dcount++;
}

//STEP TRAJECTORY WALKING
void Motion4step::StepTrajectoryWalking(int fMth){

    motions = fMth;
    if(theta > 180){
        for(i=0; i<8; i++){ lastPatternMotion[i] = PatternMotion[step][i]; } //take motion pattern matrix
            lastAngleBody = aBody;
        step ++;
        if(step > 4){
            step = 1;
            //CountSTART++;
            //if(CountSTART>=100) CountSTART=100;
        } //back to step 1
        if(step==2||step==4)MotionCounter(fMth); // motion acc & decc
        theta = t;
        if(fMth == 210 && step == 4){ //sit
            //state = READY;
        }
    }

    LengthX1 = (float)(PatternMotion[step][0] - lastPatternMotion[0]);
    LengthY1 = (float)(PatternMotion[step][1] - lastPatternMotion[1]);
    LengthZ1 = (float)(PatternMotion[step][2] - lastPatternMotion[2]);
    Heading1 = (float)(PatternMotion[step][3] - lastPatternMotion[3]);
    LengthX2 = (float)(PatternMotion[step][4] - lastPatternMotion[4]);
    LengthY2 = (float)(PatternMotion[step][5] - lastPatternMotion[5]);
    LengthZ2 = (float)(PatternMotion[step][6] - lastPatternMotion[6]);
    Heading2 = (float)(PatternMotion[step][7] - lastPatternMotion[7]);

    if(step == 1 || step ==3){
        yFoot1 = sin((theta/2) / PHI) * LengthY1 + (float)lastPatternMotion[1];
        yFoot2 = sin((theta/2) / PHI) * LengthY2 + (float)lastPatternMotion[5];
        zFoot2 = sin((theta/2) / PHI) * LengthZ2 + (float)lastPatternMotion[6];
        zFoot1 = sin((theta/2) / PHI) * LengthZ1 + (float)lastPatternMotion[2];
    }else{
        yFoot1 = (1 - sin(((theta/2)+90) / PHI)) * LengthY1 + (float)lastPatternMotion[1];
        yFoot2 = (1 - sin(((theta/2)+90) / PHI)) * LengthY2 + (float)lastPatternMotion[5];
        zFoot2 = (1 - sin(((theta/2)+90) / PHI)) * LengthZ2 + (float)lastPatternMotion[6];
        zFoot1 = (1 - sin(((theta/2)+90) / PHI)) * LengthZ1 + (float)lastPatternMotion[2];
    }
    xFoot1 = ((0 - cos(theta / PHI)+ 1)/2) * LengthX1 + (float)lastPatternMotion[0];
    xFoot2 = ((0 - cos(theta / PHI)+ 1)/2) * LengthX2 + (float)lastPatternMotion[4];
    hFoot2 = (theta / 180) * Heading2 + (float)lastPatternMotion[7];
    hFoot1 = (theta / 180) * Heading1 + (float)lastPatternMotion[3];

    // if (step == 2 || step == 3 || step == 4){
    //     HipH = (((0 - cos(theta / PHI)+ 1)/2) * LengthX1 + (float)lastPatternMotion[0])/4;
    // } else{
    //     HipH = -(((0 - cos(theta / PHI)+ 1)/2) * LengthX2 + (float)lastPatternMotion[4])/4;
    // }
    // HipZ = (zFoot1+zFoot2)/-2;
    AngleBody = (theta / 180) * (aBody - lastAngleBody) + lastAngleBody;

    kinematics.CoordinateKaki(xFoot1, yFoot1, zFoot1, hFoot1, xFoot2, yFoot2, zFoot2, hFoot2, AngleBody);
    //ROS_INFO("Kanan Y : %g | Kiri Y : %g || Kanan Z : %g | Kiri Z : %g", yFoot1,yFoot2,zFoot1,zFoot2);

    theta+=t;
    // if(step == 1 || step == 3){
    //     theta+=t;
    // }else{
    //     if(theta>0 && theta<=90)theta+=t;
    //     else if(theta>90&&theta<=108) theta+=7.7;//9/7.7
    //     else if(theta>108&&theta<=126)theta+=6.4;//7/6.4
    //     else if(theta>126&&theta<=144)theta+=4;//5/4.2
    //     else if(theta>144&&theta<=162)theta+=2;//4/2.5
    //     else if(theta > 162 && theta <= 180)theta+=1;
    //     else theta+=t;
    // }
    //ROS_INFO("9[%1.2f]",kinematics.AngleJointAll[9]);

    //Legs Servo Coordinate
    for(i=6;i<18;i++){
        if( i == 6 || i == 7 || i == 8 || i == 9 || i == 10 || i == 12 || i == 14){
            GoalPosition[i] = (int)(DefaultServo[i] - (int)((kinematics.AngleJointAll[i+1]*4096)/360));
        }else{
            GoalPosition[i] = (int)(DefaultServo[i] + (int)((kinematics.AngleJointAll[i+1]*4096)/360));
        }
    }

    //Arm Servo Coordinate
    for(i=0;i<6;i++){ 
      GoalPosition[i]=(int)(DefaultServo[i]);
    }

    //ROS_INFO("9[%d]11[%d]13[%d]15[%d]s[%d]th[%d]t[%1.2f]",GoalPosition[8],GoalPosition[10],GoalPosition[12],GoalPosition[14],step,theta,t);
}

void Motion4step::BasicMotionFSM(){
  StepTrajectoryWalking(motion);
  // if(state==RELAX){
  //   //HeadManual=ON;
  // }else if(state==STANCE){
  //   //FallDetect();
  // }else if(state==WALK){
  //   //parsingData();
  //   if(motion <= 9 && motion > 0){
  //     //Action = motion;
  //     motion = 0;

  //   } else {
  //     //Action = motion;
  //   }
  //   sleep(8);
  //   //collectDXLkaki();
  //   //sprintf(kata,"%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d\n",dataPosDXL[1],dataPosDXL[2],dataPosDXL[3],dataPosDXL[4],dataPosDXL[5],dataPosDXL[6],dataPosDXL[7],dataPosDXL[8],dataPosDXL[9],dataPosDXL[10],dataPosDXL[11],dataPosDXL[12]);
  //   //USART_puts1(kata);
  //   //FallDetect();
  // }else if(state==SIT){
  // }else if(state==KICK){
  //   //StepTrajectoryKicking(KickMode);
  // }else if(state==FALL){
  //   //Falling();
  // }

}

void Motion4step::ChangeDataAction(int fMth){
    parameter.getparam_action(fMth,sMotion,max_step,t_action);
}

void Motion4step::StepTrajectoryKicking(){
    int i;
    if(theta>180){
        for(i=0; i<8; i++) lastPatternMotion[i] = PatternMotion[step][i];
        theta = 0; step++;
        if(step>4){
            step=1;
        }

        if(step==3||step==1){
            ReadyToKick=true; lastfMth=0; cKick=0; dcount = 16; counter = 2;
        }
    }

    LengthX1 = (float)(PatternMotion[step][0] - lastPatternMotion[0]);
    LengthY1 = (float)(PatternMotion[step][1] - lastPatternMotion[1]);
    LengthZ1 = (float)(PatternMotion[step][2] - lastPatternMotion[2]);
    Heading1 = (float)(PatternMotion[step][3] - lastPatternMotion[3]);
    LengthX2 = (float)(PatternMotion[step][4] - lastPatternMotion[4]);
    LengthY2 = (float)(PatternMotion[step][5] - lastPatternMotion[5]);
    LengthZ2 = (float)(PatternMotion[step][6] - lastPatternMotion[6]);
    Heading2 = (float)(PatternMotion[step][7] - lastPatternMotion[7]);

    if(step == 1 || step ==3){
        yFoot1 = sin((theta/2) / PHI) * LengthY1 + (float)lastPatternMotion[1];
        yFoot2 = sin((theta/2) / PHI) * LengthY2 + (float)lastPatternMotion[5];
        zFoot2 = sin((theta/2) / PHI) * LengthZ2 + (float)lastPatternMotion[6];
        zFoot1 = sin((theta/2) / PHI) * LengthZ1 + (float)lastPatternMotion[2];
    }else{
        yFoot1 = (1 - sin(((theta/2)+90) / PHI)) * LengthY1 + (float)lastPatternMotion[1];
        yFoot2 = (1 - sin(((theta/2)+90) / PHI)) * LengthY2 + (float)lastPatternMotion[5];
        zFoot2 = (1 - sin(((theta/2)+90) / PHI)) * LengthZ2 + (float)lastPatternMotion[6];
        zFoot1 = (1 - sin(((theta/2)+90) / PHI)) * LengthZ1 + (float)lastPatternMotion[2];
    }
    xFoot1 = ((0 - cos(theta / PHI)+ 1)/2) * LengthX1 + (float)lastPatternMotion[0];
    xFoot2 = ((0 - cos(theta / PHI)+ 1)/2) * LengthX2 + (float)lastPatternMotion[4];
    hFoot2 = (theta / 180) * Heading2 + (float)lastPatternMotion[7];
    hFoot1 = (theta / 180) * Heading1 + (float)lastPatternMotion[3];

    kinematics.CoordinateKaki(xFoot1, yFoot1, zFoot1, hFoot1, xFoot2, yFoot2, zFoot2, hFoot2, AngleBody);
    //ROS_INFO("Kanan Y : %g | Kiri Y : %g || Kanan Z : %g | Kiri Z : %g", yFoot1,yFoot2,zFoot1,zFoot2);

    theta+=t;

    //Legs Servo Coordinate
    for(i=6;i<18;i++){
        if( i == 6 || i == 7 || i == 8 || i == 9 || i == 10 || i == 12 || i == 14){
            GoalPosition[i] = (int)(DefaultServo[i] - (int)((kinematics.AngleJointAll[i+1]*4096)/360));
        }else{
            GoalPosition[i] = (int)(DefaultServo[i] + (int)((kinematics.AngleJointAll[i+1]*4096)/360));
        }
    }

    //Arm Servo Coordinate
    for(i=0;i<6;i++){ 
      GoalPosition[i]=(int)(DefaultServo[i]);
    }
}

void Motion4step::TrajectoryShooting(int fMth){
    int i;
    if(cKick>=1 && cKick<120){
        StepTrajectoryWalking(0);
        if(cKick>=80){
            //
            //
        }
    }else if(cKick==120){
        for(i = 10; i < 18 ; i++){
                DS_Origin[i] = GoalPosition[i];
        }
        //EnStabil = ON;
        ChangeDataAction(fMth);step=1;theta=0;T=0;lastfMth = 0;cKick=147; 
        fprintf(stderr, "ChangeDataAction::fMth: %d\n", fMth);
    }else if(cKick==148){
        cKick=147;
        for(i=0; i<18; i++){
            Selisih=sMotion[i][step]-sMotion[i][step-1];
            GoalPosition[i]=(int)(((VCos[T]*Selisih)/2000)+sMotion[i][step-1]+DefaultServo[i]);
            if(i==6){
              ROS_INFO("Selisih: %d, GP7: %d",Selisih,GoalPosition[6]);
            }
        }

        T+=t_action[step];
        if(T>180){
          fprintf(stderr, "Hit!!\n");
            T=0; step++;
            if(step > 6){
                cKick=148;
            }
        }
    }else if(cKick==210){
        for(i = 0; i < 18 ; i++){
            DS_Origin[i] = DS_Backup[i];
        }
        state=WALK; counter=0; dcount=16; step = 1; ReadyToKick=false;
        ChangeDataMotion(0);
    }
    fprintf(stderr, "cKick: %d, fMth:%d, T:%d, step:%d,\n",cKick,fMth,T,step);
    cKick++;
}

// void Motion4step::StepTrajectoryAction(int fMth)
// {
//     motions = fMth;
//     if(fMth!=lastfMth){
//       ChangeDataAction(fMth);
//       //m=0;
//     }   
//     for(step=m;step<max_step;step++){
//       T=0;
//       t = t_action[step];
//       if(t == 0) t = 1;
//       //ROS_INFO("Time Step %d : %d -> t : %d", step, t_action[step], t);
//       while(T<=180)
//       {
//         for(i=0;i<=id_max;i++){
//           Selisih=sMotion[i][step]-sMotion[i][step-1];
//           AnglePosition[i]=(int)((((float)VCos[T]/1000)*Selisih)/2+sMotion[i][step-1]);
//         }
//         ROS_INFO("%g %g",AnglePosition[18],AnglePosition[19]);
//         //motor.moveMotor_degAll(AnglePosition,1,20);
// 		Motion4step::motion_publish();
//         T+=t;
//         //ROS_INFO("t = %d",t);
//         //ROS_INFO("T = %d",T);
//         //ROS_INFO("fmth = %d",fMth);
//         //ROS_INFO("STEP = %d",step);
//         //ROS_INFO("MAX_STEP = %d", max_step);
//         usleep(15000);
//       }
//       m=1;
//       /*

//       if(fMth==100){//Give Flag
//         if(step==1){t=4;}
//         else if(step==2){t=4;usleep(500000);}
//         else if(step==3){t=4;usleep(100000);}
//         else if(step==4){t=2;usleep(30000000);}
//         else if(step==5){t=4;usleep(100000);}
//       } else if(fMth==101){//Give Flag
//         if(step==1){t=1;}
//         else if(step==2){t=1;}
//       } else if(fMth==102){//Give Flag
//         if(step==1){t=1;}
//         else if(step==9){t=1;}
//       }*/
//     }
//     l=max_step;
//     //if(max_step==1||max_step==5){m=2;max_step=3;}
//     //else if(max_step==3){max_step=4;max_step=5;}
//     lastfMth=fMth;
// }

#endif // MOTION_H

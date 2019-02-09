/* 
EROS Motion Control 2018
Bismillah lulus 2019 !!

Authors: Aulia Khilmi Rizgi 
*/

#include "motion/motion_control.h"

MotionControl::MotionControl()
    :node_handle_(""),
     priv_node_handle_("~")
{
    robot_name_   = priv_node_handle_.param<std::string>("robot_name", "eros");
    pub_period_ = priv_node_handle_.param<double>("publish_period", 0.008333f);

    initPublisher();
    initSubscriber();
    initSerial();

    ROS_INFO("motion_control [node] : Init OK!");
}

MotionControl::~MotionControl(){
    fprintf(stderr,"\nDisabling torque..\n");
    setDXLData1(254,24,0);
    setDXLData1(254,24,0);
    ros::shutdown();
}

void MotionControl::initSerial(){
  if(impUSBRX() == 1)
    fprintf(stderr,".. USBRX.eros Loaded..\n");
  else fprintf(stderr,".. USBRX.eros failed to load..\n");
  char system_check[100];
  sprintf(system_check,"stty -F %s 1000000",USBRX);
  fprintf(stderr,"%s\n\n",system_check);
  system(system_check);
  system(system_check);
  system(system_check);
}

void MotionControl::InertialFeedback(){
    tresh_x=0;
    tresh_y=0;
    MassHead=0;

    if(control_enable){
        err_y=acc1-tresh_y;
        Y=err_y*3+(err_y-lastErr_y)*5+(err_y+lastErr_y)*2;

        err_x=tresh_x-acc2;

        pI -= ((float)err_x + (float)(err_x-lastErr_x)*3)/35;
        if(err_x > -7 && err_x < 5)pI=0;

        D_x=(err_x-lastErr_x);
        if(D_x<10&&D_x>-10){
            XX=err_x*7+D_x*10;//+(err_x+lastErr_x)*2;
        }else if(D_x<20&&D_x>-20){
            XX=err_x*6+D_x*50;//+(err_x+lastErr_x)*2;
        }else{
            XX=err_x*5+D_x*100;//+(err_x+lastErr_x)*2;
        }
        
        if(XX>400)XX=400;
        else if(XX<-400)XX=-400; 

        if(Xgyro>0){
            Xgyro=gyro1/3+(gyro1-lastGyroX)/2;
        }else Xgyro=gyro1/2+(gyro1-lastGyroX)/4;

        Ygyro=gyro2/2+(gyro2-lastGyroY)/8;

        //MassHead = (2400-GoalPosition[18])/80;
        //if(MassHead <= 0)MassHead = 0;
        //else if(MassHead >= 16) MassHead = 16;
            
        if(Xgyro > 200)Xgyro = 200;
        else if(Xgyro < -200)Xgyro = -200;
    }

    DefaultServo[0]=DS_Origin[0]+(XX);     //ID1
    DefaultServo[1]=DS_Origin[1]-(XX);     //ID2
    DefaultServo[2]=DS_Origin[2]-(Y);      //ID3
    if(DefaultServo[2]<DS_Origin[2]-200)DefaultServo[2]=DS_Origin[2]-200;
    DefaultServo[3]=DS_Origin[3]-(Y);      //ID4
    if(DefaultServo[3]>DS_Origin[3]+200)DefaultServo[3]=DS_Origin[3]+200;
    DefaultServo[4]=DS_Origin[4]-(XX*3/3);   //ID5
    DefaultServo[5]=DS_Origin[5]+(XX*3/3);   //ID6
            
    if(pI != 0){
        DefaultServo[10]=DS_Origin[10]+Xgyro*1/4 - XX/5 - pI/2;//#21/5/2016 "*3/4 - XX/6   //ID11
        DefaultServo[11]=DS_Origin[11]-Xgyro*1/4 + XX/5 + pI/2;//#21/5/2016                //ID12
        DefaultServo[12]=DS_Origin[12]-Xgyro*3/4 - XX/6 - pI/2;//#21/5/2016 "*5/4 - XX/6   //ID13
        DefaultServo[13]=DS_Origin[13]+Xgyro*3/4 + XX/6 + pI/2;//#21/5/2016                //ID14
        DefaultServo[14]=DS_Origin[14]+Xgyro*3/4   + MassHead + pI;// - val;           //ID15
        DefaultServo[15]=DS_Origin[15]-Xgyro*3/4 - MassHead - pI;// + val;             //ID16
    } else {
        DefaultServo[10]=DS_Origin[10]+Xgyro*1/4 - XX/6;//#21/5/2016   "Xgyro*3/4          //ID11
        DefaultServo[11]=DS_Origin[11]-Xgyro*1/4 + XX/6;//#21/5/2016                       //ID12 
        DefaultServo[12]=DS_Origin[12]-Xgyro*3/4 - XX/6;//#21/5/2016   "Xgyro*5/4          //ID13
        DefaultServo[13]=DS_Origin[13]+Xgyro*3/4 + XX/6;//#21/5/2016                       //ID14
        DefaultServo[14]=DS_Origin[14]+Xgyro*1/4   + MassHead - XX/4;// - val*3;       //ID15
        DefaultServo[15]=DS_Origin[15]-Xgyro*1/4 - MassHead + XX/4;// + val*3;         //ID16
    }   
    if(Ygyro>0){
        DefaultServo[8]  = DS_Origin[8]+Ygyro;        //ID9
        DefaultServo[16] = DS_Origin[16]+Ygyro;       //ID17
        DefaultServo[9]  = DS_Origin[9];              //ID10
        DefaultServo[17] = DS_Origin[17];             //ID18
    }else{
        DefaultServo[8]  = DS_Origin[8];              //ID9
        DefaultServo[16] = DS_Origin[16];             //ID17
        DefaultServo[17] = DS_Origin[17]+Ygyro;       //ID18
        DefaultServo[9]  = DS_Origin[9]+Ygyro;        //ID10
    }

    lastErr_x=err_x; 
    lastErr_y=err_y;
    lastGyroX=gyro1;
    lastGyroY=gyro2;
    //ROS_INFO("[%1.2f][%1.2f][%1.2f][%1.2f][%1.2f]",pI,Xgyro,Ygyro,XX,Y);
}

void MotionControl::razorCallback(const robotcontrol::Razor::ConstPtr &msg){
    acc1 = msg->acc1;
    acc2 = msg->acc2;
    gyro1 = msg->gyro1;
    gyro2 = msg->gyro2;

    //-------------------------
    //Fall Detection// DataJatuh_total as parameter for fallDetect
    countJatuh++;
    DataJatuh+=acc2;
    if(countJatuh>=10){
      DataJatuh_total=(DataJatuh/10);
      countJatuh=DataJatuh=0;
    }
    //-------------------------

    //ROS_INFO("[%d][%d][%d][%d]",acc1,acc2,gyro1,gyro2);
}

void MotionControl::motionCallback(const robotcontrol::Motion::ConstPtr &msg){
	if(state!=STANDUP){
        motion = msg->motion;
        state  = msg->state;
        headX  = msg->headX;
        headY  = msg->headY;
    }
	//ROS_INFO("[%d][%d][%d][%d]",motion,state,headX,headY);
    ChangeState();
	//Head Controller
    if(HeadControl){
        GoalPosition[18]=headY;
        GoalPosition[19]=headX;
    }
}

void MotionControl::initPublisher(){
    joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/goal_joint_position", 10);
    frequency_counter_ = node_handle_.advertise<std_msgs::Int32>(robot_name_ + "/frequency_counter", 10);
    time_counter_ = node_handle_.advertise<std_msgs::Int32>(robot_name_ + "/time_counter", 10);
}

void MotionControl::initSubscriber(){
    razor_sub_ = node_handle_.subscribe("/robotcontrol/razor", 10, &MotionControl::razorCallback, this);
    
    //Grafik servo
    dservo = node_handle_.subscribe("/setservo", 10, &MotionControl::setservo, this);

    //Motion subscribe
    motion_sub_ = node_handle_.subscribe("/robotcontrol/motion", 10, &MotionControl::motionCallback, this);
}

void MotionControl::basicMotion(){
    //ROS_INFO("[%d][%d][%d]",goal_position[0],goal_position[1],goal_position[2]);
    syncWrite();
}

void MotionControl::publishCallback(const ros::TimerEvent&){
  	secs =ros::Time::now().toSec();
	secs = secs % 2;
  	//fprintf(stderr,"Timelapsed: %d\n",secs);

  	if(secs!=last_secs){
  		std_msgs::Int32 a,b;
  		a.data=freq_count;
  		b.data=time_count;
    	frequency_counter_.publish(a);
    	time_counter_.publish(b);
    	freq_count=0;
    	time_count++;
  	}

    freq_count++;
    last_secs=secs;
}

int MotionControl::configure_port(int fd){
  struct termios port_settings;

  cfsetispeed(&port_settings, B1000000);    // set baud rates
  cfsetospeed(&port_settings, B1000000);

  memset(&port_settings,0,sizeof(port_settings));
  port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  port_settings.c_cc[4]=0;
  tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port

  return(fd);
}

void MotionControl::syncWrite(){
  Data *pdata = (struct Data *) malloc(sizeof(*pdata));
  dat[0]= (0xFF); //header 1
  dat[1]= (0xFF); //header 2
  dat[2]= 0xFE;        
  dat[4]= 0x83; //instruksi sinkronize write
  dat[5]= 0x1E; //address
  dat[6]= 0x02;
  for(iii=0;iii<20;iii++){
    dat[7+(3*iii)] = iii+1;
    dat[8+(3*iii)] = GoalPosition[iii] & 0x00ff;
    dat[9+(3*iii)] = GoalPosition[iii] >> 8;
  }
  dat[3]=64;
  for(iii=2;iii<=66;iii++){
    y+=dat[iii];
  }
  dat[67]=(~y);

  close(fdGrk); //Close serial communication, to ensure that serial is closed before we open
  fdGrk = open(USBRX, O_RDWR | O_NOCTTY | O_NONBLOCK);
  configure_port(fdGrk);

  for(iii=0;iii<=67;iii++){
    pdata->data=dat[iii];
    write(fdGrk, pdata, 1);
  }y = 0;
}

//set for Single DXL Data
void MotionControl::setDXLData1(unsigned int ID, unsigned int datae, unsigned int GoalPos){
  Data *pdata = (struct Data *) malloc(sizeof(*pdata));
  unsigned char x,y;
  close(fdGrk); //Close serial communication, to ensure that serial is closed before we open
  fdGrk = open(USBRX, O_RDWR | O_NOCTTY | O_NONBLOCK);
  configure_port(fdGrk);
  dat[0]=(0xFF);
  dat[1]=(0xFF);  y=ID;             x=y;
  dat[2]=y;       y=4;              x+=y;
  dat[3]=y;       y=0x03;           x+=y;
  dat[4]=y;       y=datae;          x+=y;
  dat[5]=y;       y=GoalPos;        x+=y;
  dat[6]=y; 
  dat[7]=(~x);
  for(iii=0;iii<8;iii++){
    pdata->data=dat[iii];
    write(fdGrk,pdata,1);
  }      
  y=0;
}


//set for High/Low DXL Data
void MotionControl::setDXLData(unsigned int ID, unsigned int datae, unsigned int GoalPos){
  Data *pdata = (struct Data *) malloc(sizeof(*pdata));
  unsigned char x,y;
  close(fdGrk); //Close serial communication, to ensure that serial is closed before we open
  fdGrk = open(USBRX, O_RDWR | O_NOCTTY | O_NONBLOCK);
  configure_port(fdGrk);
  dat[0]=(0xFF);
  dat[1]=(0xFF);  y=ID;             x=y;
  dat[2]=y;       y=5;              x+=y;
  dat[3]=y;       y=0x03;           x+=y;
  dat[4]=y;       y=datae;          x+=y;
  dat[5]=y;       y=GoalPos & 255;  x+=y;
  dat[6]=y;       y=GoalPos >> 8;   x+=y;
  dat[7]=y; 
  dat[8]=(~x);
  for(iii=0;iii<=8;iii++){
    pdata->data=dat[iii];
    write(fdGrk,pdata,1);
  }      
  y=0;
}

void MotionControl::setservo(const std_msgs::Int32::ConstPtr &msg){
    int sdata;
    sdata=msg->data;
    setDXLData(20,30,sdata);
    ROS_INFO("Set position to: %d",sdata);
}

void MotionControl::ChangeState(){
	if(state==!READY && last_state==READY){
		setDXLData(254,32,1023);
		setDXLData(254,32,1023);
		setDXLData(254,32,1023);
	}else
    if(state==WALK && last_state==KICK){
        ROS_INFO("Walk Again..");
    }

	last_state=state;
}

void MotionControl::State(){
    InertialFeedback();
	if(state==READY){
        m4.FallDetect();
		m4.BasicMotionFSM();
	}else 
	if(state==WALK){
        m4.FallDetect();
		m4.StepTrajectoryWalking(motion);
        basicMotion();
	}else
	if(state==KICK){
        m4.FallDetect();
        if(!ReadyToKick){
            m4.StepTrajectoryKicking();
        }else{
            m4.TrajectoryShooting(motion);
        }
        basicMotion();
	}else
	if(state==FALL){
        m4.Falling();
	}else
    if(state==STANDUP){
        if(standup_phase==0){
            StandUp();
            counter=0;dcount=16;cMotion=0;
        }else
        if(standup_phase==1){
            m4.StepMotion();
            basicMotion();
        }else
        if(standup_phase==2){
            StandUp_End();
        }
    }
}

void MotionControl::StandUp(){
    HeadControl     =OFF;
    control_enable  =OFF;
    setDXLData1(254,24,0);
    setDXLData1(254,24,0);
    sleep(1000);
    setDXLData1(254,24,1);
    setDXLData1(254,24,1);
    sleep(100);

    m4.FallDetect();
    state=STANDUP;
    sleep(50);
    setDXLData1(19,26,0);
    setDXLData1(19,28,1);
    if(Fall==10){
        setDXLData(1,30,DS_Origin[0]-500);
        setDXLData(3,30,DS_Origin[2]+1000);
        setDXLData(10,30,DS_Origin[9]-500);
        setDXLData(1,30,DS_Origin[0]-500);
        setDXLData(3,30,DS_Origin[2]+1000);
        setDXLData(10,30,DS_Origin[9]-500); 
        sleep(1200);
    }else
    if(Fall==20){
        setDXLData(2,30,DS_Origin[1]+500);
        setDXLData(4,30,DS_Origin[3]-1000);
        setDXLData(9,30,DS_Origin[8]+500);
        setDXLData(2,30,DS_Origin[1]+500);
        setDXLData(4,30,DS_Origin[3]-1000);
        setDXLData(9,30,DS_Origin[8]+500);
        sleep(1200);
    }else
    if(Fall==30){
        setDXLData(1,30,DS_Origin[0]-500);
        setDXLData(3,30,DS_Origin[2]+1000);
        setDXLData(10,30,DS_Origin[9]-500);
        setDXLData(11,30,DS_Origin[10]+300);
        setDXLData(13,30,DS_Origin[12]-600);
        setDXLData(15,30,DS_Origin[14]-300);
        setDXLData(1,30,DS_Origin[0]-500);
        setDXLData(3,30,DS_Origin[2]+1000);
        setDXLData(10,30,DS_Origin[9]-500);
        setDXLData(11,30,DS_Origin[10]+300);
        setDXLData(13,30,DS_Origin[12]-600);
        setDXLData(15,30,DS_Origin[14]-300);
        sleep(1200);
    }else
    if(Fall==40){
        setDXLData(2,30,DS_Origin[1]+500);
        setDXLData(4,30,DS_Origin[3]-1000);
        setDXLData(9,30,DS_Origin[8]+500);
        setDXLData(12,30,DS_Origin[11]-300);
        setDXLData(14,30,DS_Origin[13]+600);
        setDXLData(16,30,DS_Origin[15]+300);
        setDXLData(2,30,DS_Origin[1]+500);
        setDXLData(4,30,DS_Origin[3]-1000);
        setDXLData(9,30,DS_Origin[8]+500);
        setDXLData(12,30,DS_Origin[11]-300);
        setDXLData(14,30,DS_Origin[13]+600);
        setDXLData(16,30,DS_Origin[15]+300);
        sleep(1200);
    }
    standup_phase=1;
}

void MotionControl::StandUp_End(){
    setDXLData(1,26,0);
    setDXLData(1,28,16);      
    setDXLData(2,26,0);
    setDXLData(2,28,16); 
    setDXLData(5,26,0);
    setDXLData(5,28,16);      
    setDXLData(6,26,0);
    setDXLData(6,28,16); 
    setDXLData(3,26,0);
    setDXLData(3,28,16);      
    setDXLData(4,26,0);
    setDXLData(4,28,16);
    control_enable  =ON;
    state=WALK;
    m4.ChangeDataMotion(0);
    standup_phase=0;
    sleep(1000);

}

int main(int argc,char** argv){
    ros::init(argc, argv, "motion_control");
    MotionControl motion_control;
    Parameter xParam;
    std_msgs::Int32 xdata;

    robotcontrol::Trajectory pose_;

    ros::NodeHandle node_handle("");

    ros::Timer publish_timer = node_handle.createTimer(ros::Duration(motion_control.getPublishPeriod()), &MotionControl::publishCallback, &motion_control);

    //Grafik frequency
    ros::Publisher freq_gen_ = node_handle.advertise<std_msgs::Int32>("/freq_gen", 10);
    ros::Publisher pose_gen_ = node_handle.advertise<robotcontrol::Trajectory>("/pose_gen", 10);

    ros::Rate loop_rate(ITERATION_FREQUENCY);
    motion_control.setDXLData(254,32,100);
    motion_control.setDXLData(254,32,100);
    motion_control.setDXLData(254,32,100);
    control_enable=true;
    while(ros::ok()){
        motion_control.State();
    	// Graph System-----------------------------------------
    	//pose_.x1=yFoot1;
    	//pose_.x2=yFoot2;
    	//pose_gen_.publish(pose_);

    	//xdata.data=1;
    	//freq_gen_.publish(xdata);
    	//xdata.data=0;
    	//freq_gen_.publish(xdata);
    	////////////////////////////////////////////////////////
      //ROS_INFO("State:%d Motion:%d Y:%d X:%d",state,motion,GoalPosition[18],GoalPosition[19]);
    	//ROS_INFO("Running Motion..");
    	loop_rate.sleep();
    	ros::spinOnce();
    }
}

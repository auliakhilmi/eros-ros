#ifndef LOADPARAMETER_H
#define LOADPARAMETER_H

#include <ros/ros.h>

using namespace std;

class Parameter{
    private:
        ros::NodeHandle n;
        int min_walk, max_walk; //untuk menampung nilai minimum dan nilai maksimum motion kaki
        int min_action, max_action;
        std::map<int, std::vector<int> > param_walk[200];
        std::map<int, std::vector<int> > param_walk_settings;
        std::map<int, std::vector<int> > param_action[200];
        std::map<int, std::vector<int> > param_action_settings;
        short int param_default_position[20];
        short int param_goal_position[20];

    public:
        Parameter();
        void load_walk_param();
        void load_action_param();
        void load_position();
        void getparam(int motion, int(&sWalk)[5][8], float &t, float &aBody);
        void getparam_action(int motion,int(&sMotion)[18][11],int &max_step, int (&t)[11]);
        void servoPosition(short int (&pos)[20], int s);
};

Parameter::Parameter()
{
    load_walk_param();
    load_action_param();
    load_position();
}

void Parameter::load_walk_param() //fungsi untuk mengambil data parameter motion walking dari ROS Parameter server
{
    std::vector<int> data;
    std::vector<int> data_t;
    bool berhasil_load;

    char buf[50];
    if(n.getParam("walking_list/min",min_walk))
        ROS_INFO("Loaded Min Motion : %d",min_walk);
    else ROS_INFO("Failed Load Min Motion");

    if(n.getParam("walking_list/max",max_walk))
        ROS_INFO("Loadad Max Motion : %d",max_walk);
    else ROS_INFO("Failed Load Max Motion");

    for(unsigned motion = min_walk; motion<= max_walk; ++motion){
        berhasil_load = true;
        for(unsigned step = 0; step<=4; ++step){
            sprintf(buf, "walking_list/motion_%d/s%d",motion,step);
            //ROS_INFO(buf);
            if(n.getParam(buf,data)){
                param_walk[motion][step] = data;
                //ROS_INFO("Loaded data walking_list/motion_%d/s%d",motion,step);
            }else{
                berhasil_load = false;
                //ROS_INFO("motion %d Gagal",motion);
                break;
            }
        }
        sprintf(buf, "walking_list/motion_%d/q",motion);
        if(n.getParam(buf,data_t)){
            param_walk_settings[motion] = data_t;
            //ROS_INFO("Loaded data walking_list/motion_%d/q",motion);
        }else berhasil_load = false;

        if(berhasil_load == true) 
            ROS_INFO("Loaded data walking_list/motion_%d/",motion);
        //else ROS_INFO("Failed to load data walking_list/motion_%d/",motion);
    }
}

void Parameter::load_position() //fungsi untuk mengambil data parameter motion walking dari ROS Parameter server
{
    int data;
    bool berhasil_load = true;
    char buf[50];
    for(unsigned id = 1; id<=20; ++id){
        sprintf(buf, "default_list/default_position/id%d",id);
        if(n.getParam(buf,data))param_default_position[id-1] = data;
        else berhasil_load = false;
        sprintf(buf, "default_list/goal_position/id%d",id);
        if(n.getParam(buf,data))param_goal_position[id-1] = data;
        else berhasil_load = false;
    }
    if(berhasil_load)ROS_INFO("Position Load Success");
    else ROS_INFO("Position Load Failed");
}

void Parameter::load_action_param() //fungsi untuk mengambil data parameter motion walking dari ROS Parameter server
{
    //std::map<std::string, std::vector<float>> map;
    std::vector<int> data;
    std::vector<int> data_t;
    bool berhasil_load;

    char buf[50];
    if(n.getParam("action_list/min",min_action)){
        ROS_INFO("Loaded Min Action : %d",min_action);
    }else ROS_INFO("Failed Load Min Action");

    if(n.getParam("action_list/max",max_action)){
        ROS_INFO("Loaded Max Action : %d",max_action);
    }else ROS_INFO("Failed Load Max Action");

    for(unsigned action = min_action; action<= max_action; action++){
        berhasil_load = true;
        for(unsigned id = 1; id<=20; id++){
            sprintf(buf, "action_list/motion_%d/id%d",action,id);
            //ROS_INFO(buf);
            if(n.getParam(buf,data)){
                //ROS_INFO("Loadad data action_list/motion_%d/id%d",action,id);
                param_action[action][id] = data;
                //ROS_INFO("Loaded data action_list/motion_%d/id%d",action,id);
            }else{
                berhasil_load = false;
                break;
                //ROS_INFO("action %d Gagal",action);
            }
        }
        sprintf(buf, "action_list/motion_%d/t",action);
        if(n.getParam(buf,data_t)){
            param_action_settings[action] = data_t;
            //ROS_INFO("Loaded data action_list/motion_%d/t",action);
        }else berhasil_load = false;

        if(berhasil_load == true) ROS_INFO("Loaded data action_list/motion_%d/",action);
        //else ROS_INFO("Failed to load data action_list/motion_%d/",action);
    }
}

void Parameter::getparam_action(int motion, int (&sMotion)[18][11],int &max_step, int (&t)[11])
{
    for(unsigned id=0; id<18; id++){
        std::vector<int> data;
        data = param_action[motion][id+1];
        max_step = data.size();
        for(unsigned step=0; step < max_step; ++step){
            sMotion[id][step] = data[step];
            //if(id == 15) ROS_INFO("Step : %d V : %g", step,data[step]);
        }
    }
    std::vector<int> data_t;
    data_t = param_action_settings[motion];
    for(int step=0; step < data_t.size(); step++){
        t[step] = data_t[step];
        //ROS_INFO("t[%d] = %d", step, t[step]);
    }
}

void Parameter::getparam(int motion, int(&sWalk)[5][8], float &t, float &aBody)
{
    for(unsigned step=0; step<=4; ++step){
        std::vector<int> data;
        data = param_walk[motion][step];
        for(unsigned i=0; i < 8; ++i){
            sWalk[step][i] = data[i];
            //if(id == 15) ROS_INFO("Step : %d V : %g", step,data[step]);
        }
    }
    std::vector<int> data_t;
    data_t = param_walk_settings[motion];
    aBody = (float)data_t[0];
    t = (float)data_t[1];
}

void Parameter::servoPosition(short int (&pos)[20], int s){
    if(s==0)
        for(unsigned i=0; i<20; i++)
            pos[i]=param_default_position[i];
    else if(s==1)
        for(unsigned i=0; i<20; i++)
            pos[i]=param_goal_position[i];
}

#endif // LOADPARAMETER_H

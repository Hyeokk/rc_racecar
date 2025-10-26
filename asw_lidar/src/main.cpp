// ------------main문 헤더 설명-----------------|
// 각각의 hpp파일은 Task1~4로 일단 구성            |
// config의 yaml파일은 task.yaml과 state.yaml   |
// task.yaml -> ROI param 등 값들 포함          |
// state.yaml -> 기존 parent.yaml파일로 스테이트  |
// :모라이때처럼 코드 작성하면 state.yaml은 없어도 됨 |
// Task1~4.hpp -> 정민이 코드 분석해서 우리식으로변환|
//--------------------------------------------|

//-----Task.hpp include-----|
#include "task/Task1.hpp" 
#include "task/Task2.hpp" 
#include "task/Task3.hpp" 
#include "task/Task4.hpp"
#include "task/Task5.hpp"
//--------------------------|

//-----Int 선언--------------------------------------------------------| 
int mission_state, Task1_state, Task2_state, Task3_state, Task4_state,Task5_state;
//--------------------------------------------------------------------|

//-----Task.hpp 단축키 설정------|
boost::shared_ptr<Task1> a;
boost::shared_ptr<Task2> b; 
boost::shared_ptr<Task3> c; 
boost::shared_ptr<Task4> d; 
boost::shared_ptr<Task5> e; 
//-----------------------------|

//---------mission state pub Callback------------------|
void stateCallback(std_msgs::Int32 state_) {
    mission_state = state_.data;
    cout << "mission_state: " << mission_state << endl;
}
//----------------------------------------------------|

//---------Vehicle Contrl pub Callback(vehi.ctrl. can't be used)----------|
// void stateCallback(const waypoint_maker::State::ConstPtr &state_msg) {
//     mission_state = state_msg->current_state;
//     cout << "mission_state: " << mission_state << endl;
// }
//------------------------------------------------------------------------|

void mission(){
    //a == 꼬깔 || b == 터널 || c == 언제든지 등장할 동적장애물 || d == 처음 가림막 || e == 정적회피
    a->run();
    //-----------After State--------------------------|
    // if(mission_state == 0) d->run();
	// else if(mission_state == 5) s->run();
    // else if(mission_state == 9) t->run();
    // else if(mission_state == 7) g->run();
    // else if(mission_state == 4) f->run();
    // else cout<<"Not in lidar state!!!"<<endl;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "asw_lidar");
    ros::NodeHandle nh_;
    ros::Subscriber sub_state_ = nh_.subscribe("state", 1, &stateCallback);

	nh_.param("/state/Task1_state", Task1_state,-1);
    nh_.param("/state/Task2_state", Task2_state,-1);
    nh_.param("/state/Task3_state", Task3_state,-1);
    nh_.param("/state/Task4_state", Task4_state,-1);
    nh_.param("/state/Task5_state", Task5_state,-1);
	
    a.reset(new Task1());
    b.reset(new Task2());
    c.reset(new Task3());
    d.reset(new Task4());
    e.reset(new Task5());
	

    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        mission();
        loop_rate.sleep();
    }

    return 0;
}
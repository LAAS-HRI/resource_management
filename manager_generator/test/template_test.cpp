#include <thread>
#include <atomic>
#include <mutex>
#include <random>
#include <condition_variable>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <resource_management/tools/topic_name.h>
#include <resource_management/PrioritiesSetter.h>

!!for data_type in message_types
#include "${{project_name}}/{data_type[0]}.h"
!!end
#include "${project_name}/CoordinationSignal.h"

std::vector<std::string> reactive_input_names = {${reactive_input_names_cs}};

double skip_rate=0.7; // for long exaustive tests, randomly drop that proportion of tests (must be in 0,1)

template <typename T>
std::string array_to_str(const std::vector<T> &v, const std::string &sep)
{
    std::ostringstream s;
    for (size_t i = 0; i < v.size() - 1; ++i)
        s << v[i] << sep;
    s << v.back();
    return s.str();
}

class MessageGeneratorBase {

protected:
    ros::Publisher publisher;
    std::string topic_name;
public:
    std::string reactive_input_name;
    virtual void sendMessage(uint prio) = 0;
};

template<class T>
class MessageGenerator: public MessageGeneratorBase
{
    public:
    MessageGenerator(ros::NodeHandle &nh, const std::string &ns, const std::string &input_name){
        topic_name = resource_management::tools::topic_name<T>(input_name,ns);
        reactive_input_name = input_name;
        publisher = nh.advertise<T>(topic_name,1,/*latch=*/true);
    }

    void sendMessage(uint prio) override{
        T msg;
        msg.priority.value=prio;
        ROS_DEBUG("publishing reactive input with prio = %i to %s\n", prio,topic_name.c_str());
        publisher.publish(msg);
        //return msg;
    }

};

class RosNodeFixture : public ::testing::Test {
    std::vector<std::shared_ptr<MessageGeneratorBase>> reactive_input_publishers;
    ros::Publisher set_priorities;

    ros::Subscriber active_buffer;

protected:
    mutable std::mutex active_buffer_mutex;
    std::string active_buffer_value;

    ros::NodeHandle nh;

public:
    enum OptionalBool{False=0,True=1,Unset=-1};
    static OptionalBool has_artificial_life;
    RosNodeFixture(){
        ROS_DEBUG("RosNodeFixture::RosNodeFixture()");
        for(auto &n : reactive_input_names){
!!for data_type in message_types
            reactive_input_publishers.emplace_back(new MessageGenerator<::${{project_name}}::{data_type[0]}>(nh,"/${{project_name}}",n));
!!end
        }
        set_priorities = nh.advertise<resource_management::PrioritiesSetter>("/${project_name}/set_priorities",10,/*latch=*/true);

        boost::function<void(std_msgs::String)> cb=[this](auto msg){
                std::lock_guard<std::mutex> lock(active_buffer_mutex);
                this->active_buffer_value = msg.data;
                };
        active_buffer = nh.subscribe<std_msgs::String>("/${project_name}/active_buffer",10,cb);

        //while(set_priorities.getNumSubscribers()<=0){
        //    std::this_thread::sleep_for(std::chrono::milliseconds(50));
        //}

        ros::spinOnce();

        resetManagerNode();
    }

    void resetManagerNode(){
        setPriorities(std::vector<signed short>(reactive_input_names.size(),2)); //normal priority
        for(auto &buf : reactive_input_publishers){
            buf->sendMessage(-2); // avoid priority
        }
        if(has_artificial_life==Unset){
            if(expectActiveBuffer("artificial_life",2.) != "artificial_life"){
                ROS_INFO("in RosNodeFixture::resetManagerNode: the tested resource manager appears to have no artificial_life. Assume that's correct.");
                has_artificial_life=False;
            }else{
                has_artificial_life=True;
            }
        }else{
            auto actual=expectActiveBuffer(expectedDefault(),2.);
            if(actual!= expectedDefault()){
                ROS_ERROR("in RosNodeFixture::resetManagerNode: we are expecting \"%s\" as default active buffer, but we got \"%s\"",expectedDefault().c_str(),actual.c_str());
            }
        }


    }

    // the minimal priorities for a message to have a chance to be considered. It depends on the existence of artificial life on the manager.
    int minimalMessagePriority(){return (has_artificial_life==True ? ::resource_management::MessagePriority::USELESS : ::resource_management::MessagePriority::AVOID);}
    int minimalMessageBuffPriority(){return (has_artificial_life==True ? ::resource_management::PrioritiesSetter::SECONDARY : ::resource_management::PrioritiesSetter::IGNORE);}
    std::string expectedDefault(){return (has_artificial_life==True ? "artificial_life" : reactive_input_names[0]);}

    size_t reactiveBufferNumber(){return reactive_input_publishers.size();}

    void setPriorities(const std::vector<signed short> &priorities){
        ::resource_management::PrioritiesSetter msg;
        msg.buffers=reactive_input_names;
        for(auto i : priorities){
            msg.values.emplace_back(i);
        }
        set_priorities.publish(msg);
        ROS_DEBUG("setting priorities to {%s}",array_to_str(priorities,",").c_str());
        ros::spinOnce();
    }

    /// returns the name of the reactive buffer the message was sent to
    std::string publish(int i, int prio){
        reactive_input_publishers[i]->sendMessage(prio);
        return reactive_input_publishers[i]->reactive_input_name;
    }
    /// unlike publish(int,int), publishName first argument (index_name) is the index in the reactive_input_names array.
    /// select_buffer parameter can be used to select the nth buffer for that name
    // (the value wraps arround, so you can pass anything, it will send something valid)
    void publishName(int index_name, int prio, int select_buffer=0){
        std::vector<std::shared_ptr<MessageGeneratorBase>> selection;
        for(auto &x : reactive_input_publishers){
            if(x->reactive_input_name == reactive_input_names.at(index_name))
                selection.push_back(x);
        }
        selection.at(select_buffer % selection.size())->sendMessage(prio);

    }

    bool compareActiveBuffer(const std::string &name){
        std::unique_lock<std::mutex> lock(active_buffer_mutex);
        return active_buffer_value == name;

    }
    std::string getActiveBuffer(){
        std::unique_lock<std::mutex> lock(active_buffer_mutex);
        return active_buffer_value;
    }
    std::string expectActiveBuffer(const std::string &name, double timeout_sec=0.5){
        if(compareActiveBuffer(name)) return name;
        double max_time=timeout_sec*1000; //ms
        int loop_wait=std::min(50.,max_time/10.); //ms
        ros::Rate rate(1000./loop_wait); //Hz
        for(size_t i=0; i<max_time / loop_wait; ++i)
        {
            ros::spinOnce();
            if(compareActiveBuffer(name)) return name;
            rate.sleep();
        }
        return getActiveBuffer();
    }

    std::random_device rd_;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen{rd_()}; //Standard mersenne_twister_engine seeded with rd()
    bool skipTest(){
        std::uniform_real_distribution<> dis(0.0, 1.0);
        return dis(gen) < skip_rate;
    }
};

RosNodeFixture::OptionalBool RosNodeFixture::has_artificial_life = RosNodeFixture::Unset;

class CoordinationSignalFixture : public RosNodeFixture {
public:

    ros::ServiceClient coord_sig_client;

    CoordinationSignalFixture(): RosNodeFixture()
    {
        coord_sig_client = nh.serviceClient<${project_name}::CoordinationSignal>("/${project_name}/coordination_signals_register");
    }
    
    ${project_name}::CoordinationSignal makeCoordinationSignal(std::string initial, double timeout, double dl_in_secs_from_now, int prio){
        ${project_name}::CoordinationSignal sig;
        sig.request.header.initial_state=std::move(initial);
        sig.request.header.timeout=ros::Duration(timeout);
        sig.request.header.begin_dead_line = ros::Time::now() + ros::Duration(dl_in_secs_from_now);
        sig.request.header.priority.value=prio;
        return sig;
    }
    void addState(${project_name}::CoordinationSignal::Request &request, const std::string &id, const std::vector<::resource_management::CoordinationSignalsTransition> & transitions = {}){
!!fmt message_types
        ${{project_name}}::CoordinationState{0[0][0]} state;
!!end
        state.header.id=id;
        state.header.transitions = transitions;
!!fmt message_types
        request.states_{0[0][0]}.push_back(state);
!!end
    }

    ::resource_management::CoordinationSignalsTransition makeTransition(std::string next_state, double timeout, double duration, std::vector<std::string> regexs){
        ::resource_management::CoordinationSignalsTransition trans;
        trans.next_state=std::move(next_state);
        trans.end_condition.timeout=ros::Duration(std::move(timeout));
        trans.end_condition.duration=ros::Duration(std::move(duration));
        trans.end_condition.regex_end_condition=std::move(regexs);
        return trans;
    }

    ${project_name}::CoordinationSignal makeSimpleCoordinationSignal(int prio){
        auto sig = makeCoordinationSignal("0",.5,.5,prio);
        addState(sig.request, "0", {makeTransition("final",0.2,0.2,{})});
        return sig;
    }
};

TEST_F(RosNodeFixture,startWithArtificialLife){
    ASSERT_EQ(expectedDefault(),expectActiveBuffer(expectedDefault(),5.)); // long wait to wait for initialization of the tested node
}
// checks that when several reactive inputs are given, with same importance
// and buffers have same priority, the first type is selected (in the order
// of declaration of those types in the manager code)
TEST_F(RosNodeFixture,samePriority){
    EXPECT_EQ(expectedDefault(),expectActiveBuffer(expectedDefault()));
    std::vector<signed short> prio(reactive_input_names.size(),3);
    setPriorities(prio);
    ros::spinOnce();
    for(size_t i = 0; i<reactiveBufferNumber(); ++i){
        auto name = publish(i,2);
        ros::spinOnce();
        EXPECT_EQ(reactive_input_names[0],expectActiveBuffer(reactive_input_names[0]));
    }
}

TEST_F(RosNodeFixture, reset){
    ASSERT_EQ(expectedDefault(),expectActiveBuffer(expectedDefault()));
}

// a reactive input is preempted by one with higher priority
// with messages of strictly same importance
// for all input priority and message importance except AVOID.
TEST_F(RosNodeFixture, reactiveInputPriorityPreempt){
    for(int base_prio=::resource_management::PrioritiesSetter::IGNORE; base_prio<=::resource_management::PrioritiesSetter::FULLFOCUS; ++base_prio){
        for (int top_prio=base_prio+1; top_prio <= ::resource_management::PrioritiesSetter::FULLFOCUS; ++top_prio){
            for(int message_prio=::resource_management::MessagePriority::AVOID; message_prio <= ::resource_management::MessagePriority::VITAL; ++message_prio){
                for(size_t top_prio_i = 0; top_prio_i < reactive_input_names.size(); ++top_prio_i){
                    if(skipTest()) continue;
                    std::vector<signed short> prio(reactive_input_names.size(),base_prio);
                    prio[top_prio_i]=top_prio;
                    setPriorities(prio);
                    ros::spinOnce();
                    for(size_t i = 0; i<reactiveBufferNumber(); ++i){
                        publish(i,message_prio);
                    }
                    ros::spinOnce();
                    if(message_prio >= minimalMessagePriority()
                            && top_prio >= minimalMessageBuffPriority()){
                        //expect top priority reactive input
                        EXPECT_EQ(reactive_input_names[top_prio_i], expectActiveBuffer(reactive_input_names[top_prio_i]))
                            << "message priority is "<<message_prio<<" and buffer priorities are {"<<array_to_str(prio,", ")<<"}";
                    }else{
                        //expect artificial_life
                        EXPECT_EQ("artificial_life", expectActiveBuffer("artificial_life"))
                            << "message priority is "<<message_prio<<" and buffer priorities are {"<<array_to_str(prio,", ")<<"}";
                    }
                }
            }
        }
    }
}

// the message with higher importance must be selected,
// whatever the reactive input priorities are (except FULLFOCUS, not tested here).
// Only test for message that should be preferred over artificial_life
TEST_F(RosNodeFixture, reactiveInputMsgImportancePreempt){
    for(int base_prio=minimalMessageBuffPriority(); base_prio<=::resource_management::PrioritiesSetter::FULLFOCUS-1; ++base_prio){
        for (int other_prio=minimalMessageBuffPriority(); other_prio <= ::resource_management::PrioritiesSetter::FULLFOCUS-1; ++other_prio){
            for(int message_prio=minimalMessagePriority(); message_prio <= ::resource_management::MessagePriority::VITAL; ++message_prio){
                for(int message_top_prio=message_prio+1; message_top_prio <= ::resource_management::MessagePriority::VITAL; ++message_top_prio){
                    for(size_t other_prio_i = 0; other_prio_i < reactive_input_names.size(); ++other_prio_i){
                        std::vector<signed short> prio(reactive_input_names.size(),base_prio);
                        prio[other_prio_i]=other_prio;
                        setPriorities(prio);
                        for(size_t msg_top_prio_i=0;msg_top_prio_i<reactive_input_names.size();++msg_top_prio_i){
                            if(skipTest()) continue;
                            std::vector<int> message_prios;
                            for(size_t i = 0; i<reactive_input_names.size(); ++i){
                                if(i==msg_top_prio_i){
                                    publishName(i,message_top_prio);
                                    message_prios.push_back(message_top_prio);
                                }else{
                                    publishName(i,message_prio);
                                    message_prios.push_back(message_prio);
                                }
                            }
                            //expect top priority message to be selected
                            EXPECT_EQ(reactive_input_names[msg_top_prio_i], expectActiveBuffer(reactive_input_names[msg_top_prio_i]))
                                << "message priorities are {"<<array_to_str(message_prios,", ")<<"} and buffer priorities are {"<<array_to_str(prio,", ")<<"}";
                        }
                    }
                }
            }
        }
    }
}

// messages passed on a fullfocus priority input are preferred over all other messages.
// Except VITAL ones (not tested)
// Except AVOID ones (not tested)
// When all other inputs have a priority < fullfocus (strictly inferior)
TEST_F(RosNodeFixture, fullFocusPreempt){
    for(int base_prio=minimalMessageBuffPriority(); base_prio<=::resource_management::PrioritiesSetter::FULLFOCUS-1; ++base_prio){
        int other_prio=::resource_management::PrioritiesSetter::FULLFOCUS;
        for(int message_prio=::resource_management::MessagePriority::USELESS; message_prio <= ::resource_management::MessagePriority::VITAL-1; ++message_prio){
            for(int message_top_prio=message_prio+1; message_top_prio <= ::resource_management::MessagePriority::VITAL-1; ++message_top_prio){
                for(size_t other_prio_i = 0; other_prio_i < reactive_input_names.size(); ++other_prio_i){
                    std::vector<signed short> prio(reactive_input_names.size(),base_prio);
                    prio[other_prio_i]=other_prio;
                    setPriorities(prio);
                    for(size_t msg_top_prio_i=0;msg_top_prio_i<reactive_input_names.size();++msg_top_prio_i){
                        std::vector<int> message_prios;
                        //all messages have same priority except one which is higher
                        //this higher message is successivelly put into each reactive input
                        for(size_t i = 0; i<reactive_input_names.size(); ++i){
                            if(i==msg_top_prio_i){
                                publishName(i,message_top_prio);
                                message_prios.push_back(message_top_prio);
                            }else{
                                publishName(i,message_prio);
                                message_prios.push_back(message_prio);
                            }
                        }
                        //expect fullfocus priority input to be selected
                        EXPECT_EQ(reactive_input_names[other_prio_i], expectActiveBuffer(reactive_input_names[other_prio_i]))
                            << "message priorities are {"<<array_to_str(message_prios,", ")<<"} and buffer priorities are {"<<array_to_str(prio,", ")<<"}";
                    }
                }
            }
        }
    }
}

// when several reactive inputs have the same priority, the selected one is the one with the 
// message with higher importance
TEST_F(RosNodeFixture, sameBufferPrioSelectMessagePrio){
    for(int base_prio=minimalMessageBuffPriority(); base_prio<=::resource_management::PrioritiesSetter::FULLFOCUS; ++base_prio){
        std::vector<signed short> prio(reactive_input_names.size(),base_prio);
        setPriorities(prio);
        for(int message_prio=minimalMessagePriority(); message_prio <= ::resource_management::MessagePriority::VITAL; ++message_prio){
            for(int message_top_prio=message_prio+1; message_top_prio <= ::resource_management::MessagePriority::VITAL-1; ++message_top_prio){
                for(size_t top_prio_i = 0; top_prio_i < reactive_input_names.size(); ++top_prio_i){
                    std::vector<int> message_prios;
                    if(skipTest()) continue;
                    for(size_t i = 0; i<reactive_input_names.size(); ++i){
                        if(i==top_prio_i){
                            publishName(i,message_top_prio);
                            message_prios.push_back(message_top_prio);
                        }else{
                            publishName(i,message_prio);
                            message_prios.push_back(message_prio);
                        }
                    }
                    //expect top priority message
                    EXPECT_EQ(reactive_input_names[top_prio_i], expectActiveBuffer(reactive_input_names[top_prio_i]))
                        << "message priority is "<<message_prio<<" and buffer priorities are {"<<array_to_str(prio,", ")<<"}";
                }
            }
        }
    }
}

// vital message on whatever input preempts any other message (including fullfocus)
TEST_F(RosNodeFixture, vitalPreemptsAny){
    for(int base_prio=minimalMessageBuffPriority(); base_prio<=::resource_management::PrioritiesSetter::FULLFOCUS; ++base_prio){
        for (int other_prio=minimalMessageBuffPriority(); other_prio <= ::resource_management::PrioritiesSetter::FULLFOCUS; ++other_prio){
            for(size_t other_prio_i = 0; other_prio_i < reactive_input_names.size(); ++other_prio_i){
                std::vector<signed short> prio(reactive_input_names.size(),base_prio);
                prio[other_prio_i]=other_prio;
                setPriorities(prio);
                for(int message_prio=minimalMessagePriority(); message_prio <= ::resource_management::MessagePriority::VITAL-1; ++message_prio){
                    int message_top_prio=::resource_management::MessagePriority::VITAL;
                    for(size_t msg_top_prio_i=0;msg_top_prio_i<reactive_input_names.size();++msg_top_prio_i){
                        if(skipTest()) continue;
                        std::vector<int> message_prios;
                        for(size_t i = 0; i<reactive_input_names.size(); ++i){
                            if(i==msg_top_prio_i){
                                publishName(i,message_top_prio);
                                message_prios.push_back(message_top_prio);
                            }else{
                                publishName(i,message_prio);
                                message_prios.push_back(message_prio);
                            }
                        }
                        //expect top priority message to be selected
                        EXPECT_EQ(reactive_input_names[msg_top_prio_i], expectActiveBuffer(reactive_input_names[msg_top_prio_i]))
                            << "message priorities are {"<<array_to_str(message_prios,", ")<<"} and buffer priorities are {"<<array_to_str(prio,", ")<<"}";
                    }
                }
            }
        }
    }
}

// test coordination signal id are unique
TEST_F(CoordinationSignalFixture,uniqueIds){
    std::vector<uint> ids;
    ids.reserve(100);
    for(size_t i = 0; i <100; ++i){
        auto sig = makeSimpleCoordinationSignal(2);
        bool srv_res = coord_sig_client.call(sig);
        EXPECT_TRUE(srv_res) << "failed to call coordination signal registration service";
        if(srv_res){
            auto search = std::find(ids.begin(),ids.end(),sig.response.id);
            EXPECT_EQ(ids.end(),search) << "coordination signal id is not unique";
            ids.push_back(sig.response.id);
        }
    }
}

// send one coord sig. meanwhile send n with different prio. check the higher prio one is selected when first one ends

// test coordination sig duration (timeout)

// test coord sig begin deadline -> send one coord sig and another one that should start before the first ended. it will not be executed

// send 1 coord sig. send a second and cancel it before it starts. check it is not executed.



int main(int argc, char *argv[]){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc,argv,"tests_${project_name}");
    ros::NodeHandle nh;
    auto res= RUN_ALL_TESTS();

    return res;
}

#pragma once
#include <stdint.h>

class Task {
    // private:
    // static Task *instance;
    // Task(){}
    // Taks(const Task &) = delete;
    // Task &operator = (const Task &) = delete;

    public : 
    // abstract class yg dapat di override
        virtual bool ready(uint32_t now) = 0;   //ready state

        virtual void run(uint32_t now) = 0;

};

// Inheritence : 
/*
* conditional task digunakan untuk mengeksekusi task dengan 
* interupsi
*/
class ConditionalTask : public Task {
    public:
        virtual bool ready(uint32_t now);   // ready state 

        inline void setReady(){runFlag = true;}

        inline void notReady(){runFlag = false;}

    protected:
        bool runFlag;
};

/*
* timing task digunakan untuk mengeksekusi task pada rate tertentu
* 
*/
class TimingTask :public Task{
public:
    inline void updateTime(uint32_t when){runTime = when;}

    virtual bool ready(uint32_t now);

    inline void setRunTime(uint32_t when){runTime = when;}

    inline void tick(uint32_t inc){runTime += inc;}

    inline uint32_t getRunTime(){return runTime;}

protected:
    uint32_t runTime;
};
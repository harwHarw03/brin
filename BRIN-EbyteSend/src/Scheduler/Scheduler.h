#pragma once
#include "Task.h"

#define NUM_TASKS(T) (sizeof(T) / sizeof(Task)) // count number of task 

class Scheduler {
    public:
        Scheduler(Task **task, uint8_t numTasks);
        void runTasks();
    
    private:
        Task **tasks;   //array of pointers
        int numTasks;
        /*
        * di program lama menggunakan struct utk save task datas
        * di sini menggunakan kelas karena tidak punya functor 
        * (line 70 scheduler.h program lama)
        */
};


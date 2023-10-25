#include <Arduino.h>
#include "Scheduler.h"

Scheduler::Scheduler(Task **_tasks, uint8_t _numTasks) : tasks(_tasks),numTasks(_numTasks) {}

/*
simple task runner from Alan
*/
void Scheduler::runTasks() {
    // while (1) {
        uint32_t now = millis();
        Task **tpp = tasks;
        for (int t = 0; t < numTasks; t++) {
            Task *tp = *tpp;
            if (tp->ready(now)) {
                tp->run(now);
                break;
            }
            tpp++;  //next task
        }
    // }
}
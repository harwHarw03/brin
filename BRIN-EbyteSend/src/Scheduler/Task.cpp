#include "Task.h"

// Virtual.

bool ConditionalTask::ready(uint32_t now) {
    return runFlag;
}

bool TimingTask::ready(uint32_t now) {  // wait
    return now >= runTime;
}
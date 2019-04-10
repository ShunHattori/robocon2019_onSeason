#include "featureManager.h"

featureManager::featureManager()
{
    globalTimer.start();
}

int featureManager::getTaskStats()
{
    return taskIsRunning;
}

void featureManager::addTask(taskList taskNum)
{
    switch (taskNum)
    {
    case TEST_1:
        runningTEST_1 = true;
        break;
    default:
        break;
    }
}

void featureManager::update()
{
    if (runningTEST_1)
    {
        runTEST_1();
    }
    else
    {
        taskIsRunning = 0;
    }
}

void featureManager::runTEST_1()
{
    static bool initial = 1;
    static long initialTime;
    if (initial)
    {
        initialTime = globalTimer.read_ms();
        initial = 0;
    }
    else
    {
        if ((globalTimer.read_ms() - initialTime) > 300)
            {
                runningTEST_1 = false;
            }
        else
        {
        }
    }
}
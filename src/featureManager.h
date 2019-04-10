#pragma once

#include "mbed.h"
#include <vector>


enum taskList
{
  TEST_1,
};

class featureManager
{
  public:
    featureManager();

    /*
    *   desc:   現在タスクが実行されているかを返す
    *   param:  none
    *   return: 1 or 0
    */
    int getTaskStats();

    /*
    *   desc:   引数の番号に割り当てられているタスクをキューに追加する
    *   param:  タスク番号
    *   return: none
    */
    void addTask(taskList);

    void update();

  private:

  Timer globalTimer;
  std::vector<int> taskQueue;
  bool taskIsRunning;

  bool runningTEST_1;
  void runTEST_1();
};
#include <iostream>
#include <windows.h>

#include "custom_share_manager.h"

int main() {
  std::cout << "Hello, world!";
  CustomShareManager::createSingleton();
  while (true) {
    unsigned char gazeStatus[0x148];
    CustomShareManager::getSingleton()->getGazeStatus(gazeStatus);
    std::cout << gazeStatus;
    Sleep(1);
  }
  return 0;
}

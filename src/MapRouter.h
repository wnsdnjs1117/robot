#ifndef MAP_ROUTER_H
#define MAP_ROUTER_H
#include <Arduino.h>

void executeBlindRun();
void goToNodeFromHub8(int node);
void returnToHub8FromNode(int node, bool cameOutForward);

#endif
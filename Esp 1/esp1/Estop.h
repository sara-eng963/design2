#pragma once

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

void configureEStopPin();
bool createEStopPublisher(rcl_node_t *node);
void destroyEStopPublisher(rcl_node_t *node);
void resetEStopPublisher();
void prepareEStopInitialPublish();
void pollEStopPublisher();

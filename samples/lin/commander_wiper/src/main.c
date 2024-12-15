/*
 * Copyright (C) Frickly Systems GmbH
 * Copyright (C) MBition GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ardep/drivers/abstract_lin.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct device *lin = DEVICE_DT_GET(DT_NODELABEL(abstract_lin0));

//static bool remote_led_target = false;
//static bool remote_led_is = false;

uint8_t drvFreqReq = 0x05;
uint16_t drvTargetPos = 0;
uint8_t drvMode = 0x02;

uint16_t drvCurrentPosition = 0;

static uint16_t upperPos = (uint16_t)60/0.010989; //degree * const calc 0.010989
static uint16_t lowerPos = (uint16_t)0;
 

//ID 0x33 = Command HPC to Wiper Drive dec: 51 8 Byte payload
static bool command_callback(struct lin_frame *frame, void *) {
  //LOG_DBG("Command callback called");
  //0: uint8 FreqReq 0 - 90 in WpCycl/Min
  //1+2: uint16 TargetPos physical value 0 - 16380 in degree calc 0.010989
  //3: uint8 Mode 0 - 2 Enum "STOP"=0, "HOLD"=1, "MOVE"=2
  //4-8: reserve
  frame->data[0] = drvFreqReq; //drvFreqReq;
  frame->data[1] = drvTargetPos >> 8; //0xFF;
  frame->data[2] = drvTargetPos & 0xFF; //0xFF;
  frame->data[3] = drvMode; //drvMode;
  LOG_DBG("Command values: %d %d %d %d", frame->data[0], frame->data[1], frame->data[2], frame->data[3]);
  //From Example: frame->data[0] = remote_led_target;

  // send unconditionally
  return true;
}

static void status_callback(const struct lin_frame *frame, void *) {
  //LOG_DBG("Status callback called");
  //0+1: uint16 drvPosition physical value 0 - 16380 in degree calc 0.010989
  //2+3: uint16 drvCurrent physical value 0 - 8000 calc 0.01 in Amps
  //4: uint8 drvTemp physical value 0 - 254 in C calc -50
  //5: uint8 drvStatus 1:IsWiping 2:IsEndingWpCycle 3:IsWpError 4:IsPositionReached 5:IsBlocked 6:IsOverheat 7:IsOverVoltage 8:IsUnderVoltage
  //6: uint4 drvLINError + uint4 drvEcuTemp 
  //LOG_INF("Recieved: %d", frame->data[5]);
  if(frame->data[5] & (1 << 1)) {
    LOG_INF("WIPERSTATUS: is Wiping waiting to change target");
  }
  else if(frame->data[5] & (1 << 2) || frame->data[5] & (1 << 3)) 
  {
    LOG_INF("WIPERSTATUS: isEndingWPCycle or isPositionReached");
    drvCurrentPosition = frame->data[1] << 8 | frame->data[0];
    LOG_INF("drvCurrentPosition: %d", drvCurrentPosition);
    if(drvCurrentPosition <=  20)
    {
      drvTargetPos = upperPos;
    }
    else
    {
      drvTargetPos = lowerPos;
    }
    LOG_INF("drvTargetPos: %d", drvTargetPos);
  }
}

int main(void) {
  LOG_INF("Hello world!");

  if (!device_is_ready(lin)) {
    LOG_ERR("Device not ready");
    return -1;
  }

  int err;
  // check if we have enough free slots for our 2 callbacks
  {
    uint8_t free_slots = 0;
    if ((err = abstract_lin_get_free_callback_slot(lin, &free_slots))) {
      LOG_ERR("Error reading free slot count. err: %d", err);
    }
    if (free_slots < 2) {
      LOG_ERR("Not enough free slots for setting up callbacks. Free_slots: %d",
              free_slots);
      return -1;
    }
  }

  // register callbacks
  //ID 0x34 = Wiper Drive status to HPC dec: 52 8 Byte payload
  if ((err = abstract_lin_register_incoming(lin, &status_callback, 0x34, 8,
                                            NULL))) {
    LOG_ERR("Error registering incoming status callback. err: %d", err);
  };
  //ID 0x33 = Command HPC to Wiper Drive dec: 51 8 Byte payload
  if ((err = abstract_lin_register_outgoing(lin, &command_callback, 0x33, 8,
                                            NULL))) {
    LOG_ERR("Error registering outgoing command callback. err: %d", err);
  };

  LOG_INF("Registered cbs");
  //trigger once
  //drvFreqReq = upperPos;
  //LOG_INF("drvFreqReq = %d", drvFreqReq);
  // toggle led and poll status
  for (;;) {
    //remote_led_target = !remote_led_target;

    k_msleep(100);
    //LOG_INF("Schedule receive status");
    abstract_lin_schedule_now(lin, 0x34);  // receive status

    k_msleep(100);
    //LOG_INF("Schedule send command");
    abstract_lin_schedule_now(lin, 0x33);  // send command
  }

  return 0;
}

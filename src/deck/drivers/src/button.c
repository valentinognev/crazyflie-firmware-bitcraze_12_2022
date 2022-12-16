

#include "deck.h"
#include "system.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "debug.h"
#include "param.h"

#define DEBUG_MODULE "BUTTON"

static uint8_t button = 0;

static void sequenceTask()
{
  systemWaitStart();

  DEBUG_PRINT("***** BUTTON Waiting for activation ...\n");


  while(1) {
    if (button==1)
    {
      DEBUG_PRINT("***** activation now ...\n");
      digitalWrite(DECK_GPIO_TX2, HIGH);
      vTaskDelay(M2T(500));
      digitalWrite(DECK_GPIO_TX2, LOW);
      button=0;
    }
    vTaskDelay(M2T(20));
  }

}


static void sequenceInit()
{
  pinMode(DECK_GPIO_TX2, OUTPUT);     // Set  pin to output
  digitalWrite(DECK_GPIO_TX2, LOW);  // OFF
  xTaskCreate(sequenceTask, "sequence", configMINIMAL_STACK_SIZE, NULL, /*priority*/3, NULL);
}

static bool sequenceTest()
{
  return true;
}

const DeckDriver sequence_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcButton",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = sequenceInit,
  .test = sequenceTest,
};


DECK_DRIVER(sequence_deck);

LOG_GROUP_START(button)
LOG_ADD(LOG_UINT8, button, &button)
LOG_GROUP_STOP(button)

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8, button, &button)
PARAM_GROUP_STOP(deck)
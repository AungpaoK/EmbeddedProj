#include "DeliveryStateMachine.h"

#include "PiCommunication.h"
#include "Sensors.h"
#include "UserInterface.h"

namespace {
  DeliveryState state = SELECT_SHELF;

  DeliveryJob jobs[2];
  uint8_t jobCount = 0;
  uint8_t currentJob = 0;
  uint8_t selectedShelf = 0;
  uint8_t enteredTable = 0;

  bool foodSeenAtTable = false;

  void showState() {
    uiShowState(
      state,
      selectedShelf,
      enteredTable,
      jobs,
      jobCount,
      currentJob
    );
  }

  void changeState(DeliveryState newState) {
    state = newState;
    showState();
  }

  bool shelfIsUsed(uint8_t shelf) {
    for (uint8_t i = 0; i < jobCount; i++) {
      if (jobs[i].shelf == shelf) {
        return true;
      }
    }
    return false;
  }

  void resetDelivery(bool notifyPi) {
    jobCount = 0;
    currentJob = 0;
    selectedShelf = 0;
    enteredTable = 0;
    foodSeenAtTable = false;
    changeState(SELECT_SHELF);

    if (notifyPi) {
      communicationSendReset();
    }
  }

  void sendCurrentJob() {
    communicationSendDelivery(jobs[currentJob]);
    changeState(TRAVELLING);
  }

  void startDelivery() {
    if (jobCount == 0) {
      return;
    }

    currentJob = 0;
    sendCurrentJob();
  }

  void finishCurrentJob() {
    communicationSendDelivered(jobs[currentJob]);
    currentJob++;

    if (currentJob < jobCount) {
      sendCurrentJob();
    } else {
      communicationSendReturnHome();
      changeState(RETURNING_HOME);
    }
  }

  void handleSelectShelf(char key) {
    if (key == '1' || key == '2') {
      selectedShelf = key - '0';

      if (shelfIsUsed(selectedShelf)) {
        selectedShelf = 0;
        uiShowShelfAlreadyUsed();
      } else {
        uiShowSelectedShelf(selectedShelf);
      }
    }
    else if (key == 'B' && selectedShelf != 0) {
      if (sensorsFoodIsPresent(selectedShelf)) {
        enteredTable = 0;
        changeState(ENTER_TABLE);
      } else {
        changeState(WAIT_FOR_FOOD);
      }
    }
  }

  void handleEnterTable(char key) {
    if (key >= '0' && key <= '9') {
      uint8_t digit = key - '0';

      // รับหมายเลขโต๊ะ 1-99 เท่านั้น
      if (enteredTable < 10) {
        enteredTable = enteredTable * 10 + digit;
        uiShowTableNumber(enteredTable);
      }
    }
    else if (key == 'C' && enteredTable > 0) {
      jobs[jobCount].shelf = selectedShelf;
      jobs[jobCount].table = enteredTable;
      jobCount++;
      changeState(READY_TO_START);
    }
  }

  void handleReady(char key) {
    if (key == 'A' && jobCount < 2) {
      selectedShelf = 0;
      enteredTable = 0;
      changeState(SELECT_SHELF);
    }
    else if (key == '#') {
      startDelivery();
    }
  }
}

void deliveryBegin() {
  resetDelivery(false);
}

void deliveryUpdate() {
  char key = uiReadKey();
  if (!key) {
    return;
  }

  // กด * เพื่อล้างงานในช่วงตั้งค่างาน
  if (key == '*' && state <= READY_TO_START) {
    resetDelivery(true);
    return;
  }

  switch (state) {
    case SELECT_SHELF:
      handleSelectShelf(key);
      break;

    case WAIT_FOR_FOOD:
      // รอ IR Sensor จึงไม่ต้องประมวลผลปุ่มอื่น
      break;

    case ENTER_TABLE:
      handleEnterTable(key);
      break;

    case READY_TO_START:
      handleReady(key);
      break;

    case TRAVELLING:
    case WAIT_FOR_PICKUP:
    case RETURNING_HOME:
      break;
  }
}

void deliveryHandleFoodChange(uint8_t shelf, bool foodPresent) {
  if (state == WAIT_FOR_FOOD &&
      shelf == selectedShelf &&
      foodPresent) {
    enteredTable = 0;
    changeState(ENTER_TABLE);
    return;
  }

  if (state == TRAVELLING &&
      shelf == jobs[currentJob].shelf &&
      !foodPresent) {
    communicationSendFoodMissing(shelf);
    uiShowFoodMissing(shelf);
    return;
  }

  if (state == WAIT_FOR_PICKUP && shelf == jobs[currentJob].shelf) {
    if (foodPresent) {
      foodSeenAtTable = true;
    }
    else if (foodSeenAtTable) {
      finishCurrentJob();
    }
  }
}

void deliveryHandlePiCommand(PiCommand command) {
  if (command == PI_ARRIVED && state == TRAVELLING) {
    foodSeenAtTable = sensorsFoodIsPresent(jobs[currentJob].shelf);
    changeState(WAIT_FOR_PICKUP);

    if (!foodSeenAtTable) {
      communicationSendFoodMissing(jobs[currentJob].shelf);
      uiShowFoodMissing(jobs[currentJob].shelf);
    }
  }
  else if (command == PI_HOME && state == RETURNING_HOME) {
    resetDelivery(false);
  }
  else if (command == PI_RESET) {
    resetDelivery(false);
  }
}

DeliveryState deliveryGetState() {
  return state;
}

/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2019 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * mem.c: Memory module. Handles one-wire, eeprom and other memory mapping memory functions over crtp link.
 */

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "config.h"
#include "crtp.h"
#include "mem.h"
#include "ow.h"
#include "eeprom.h"

#include "ledring12.h"
#include "locodeck.h"
#include "crtp_commander_high_level.h"
#include "lighthouse_position_est.h"
#include "usddeck.h"

#include "console.h"
#include "assert.h"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "static_mem.h"

#if 0
#define MEM_DEBUG(fmt, ...) DEBUG_PRINT("D/log " fmt, ## __VA_ARGS__)
#define MEM_ERROR(fmt, ...) DEBUG_PRINT("E/log " fmt, ## __VA_ARGS__)
#else
#define MEM_DEBUG(...)
#define MEM_ERROR(...)
#endif


// Maximum log payload length
#define MEM_MAX_LEN 30

#define MEM_SETTINGS_CH     0
#define MEM_READ_CH         1
#define MEM_WRITE_CH        2

#define MEM_CMD_GET_NBR     1
#define MEM_CMD_GET_INFO    2

// The first part of the memory ids are static followed by a dynamic part
// of one wire ids that depends on the decks that are attached
#define EEPROM_ID       0x00
#define LEDMEM_ID       0x01
#define LOCO_ID         0x02
#define TRAJ_ID         0x03
#define LOCO2_ID        0x04
#define LH_ID           0x05
#define TESTER_ID       0x06
#define USD_ID          0x07
#define OW_FIRST_ID     0x08

#define STATUS_OK 0

#define MEM_TYPE_EEPROM 0x00
#define MEM_TYPE_OW     0x01
#define MEM_TYPE_LED12  0x10
#define MEM_TYPE_LOCO   0x11
#define MEM_TYPE_TRAJ   0x12
#define MEM_TYPE_LOCO2  0x13
#define MEM_TYPE_LH     0x14
#define MEM_TYPE_TESTER 0x15
#define MEM_TYPE_USD    0x16

#define MEM_LOCO_INFO             0x0000
#define MEM_LOCO_ANCHOR_BASE      0x1000
#define MEM_LOCO_ANCHOR_PAGE_SIZE 0x0100
#define MEM_LOCO_PAGE_LEN         (3 * sizeof(float) + 1)

#define LOCO_MESSAGE_NR_OF_ANCHORS 8

#define MEM_LOCO2_ID_LIST          0x0000
#define MEM_LOCO2_ACTIVE_LIST      0x1000
#define MEM_LOCO2_ANCHOR_BASE      0x2000
#define MEM_LOCO2_ANCHOR_PAGE_SIZE 0x0100
#define MEM_LOCO2_PAGE_LEN         (3 * sizeof(float) + 1)

#define MEM_TESTER_SIZE            0x1000

//Private functions
static void memTask(void * prm);
static void memSettingsProcess(CRTPPacket* p);
static void memWriteProcess(CRTPPacket* p);
static void memReadProcess(CRTPPacket* p);
static uint8_t handleLocoMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* dest);
static uint8_t handleLoco2MemRead(uint32_t memAddr, uint8_t readLen, uint8_t* dest);
static void createNbrResponse(CRTPPacket* p);
static void createInfoResponse(CRTPPacket* p, uint8_t memId);
static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]);

static uint8_t handleEepromRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData);
static uint8_t handleEepromWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData);
static uint8_t handleLedMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData);
static uint8_t handleLedMemWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData);
static uint8_t handleTrajectoryMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData);
static uint8_t handleTrajectoryMemWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData);
static uint8_t handleLighthouseMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData);
static uint8_t handleLighthouseMemWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData);
static uint8_t handleMemTesterRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData);
static uint8_t handleMemTesterWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData);
static uint32_t memTesterWriteErrorCount = 0;
static uint8_t memTesterWriteReset = 0;
static uint8_t handleUsdMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData);
static uint8_t handleOneWireMemRead(uint8_t memId, uint32_t memAddr, uint8_t readLen, uint8_t* startOfData);
static uint8_t handleOneWireMemWrite(uint8_t memId, uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData);

static bool isInit = false;

static uint8_t nbrOwMems = 0;
static OwSerialNum serialNbr;
static const OwSerialNum eepromSerialNum =
{
  .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, EEPROM_I2C_ADDR}
};
static const uint8_t noData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static CRTPPacket packet;

STATIC_MEM_TASK_ALLOC(memTask, MEM_TASK_STACKSIZE);

void memInit(void)
{
  if(isInit) {
    return;
  }

  if (owScan(&nbrOwMems)) {
    isInit = true;
  } else {
    isInit = false;
  }

  //Start the mem task
  STATIC_MEM_TASK_CREATE(memTask, memTask, MEM_TASK_NAME, NULL, MEM_TASK_PRI);
}

bool memTest(void) {
  return isInit;
}

static void memTask(void* param) {
	crtpInitTaskQueue(CRTP_PORT_MEM);

	while(1) {
		crtpReceivePacketBlock(CRTP_PORT_MEM, &packet);

		switch (packet.channel) {
      case MEM_SETTINGS_CH:
        memSettingsProcess(&packet);
        break;
      case MEM_READ_CH:
        memReadProcess(&packet);
        break;
      case MEM_WRITE_CH:
        memWriteProcess(&packet);
        break;
      default:
        // Do nothing
        break;
		}
	}
}

static void memSettingsProcess(CRTPPacket* p) {
  switch (p->data[0]) {
    case MEM_CMD_GET_NBR:
      createNbrResponse(p);
      crtpSendPacket(p);
      break;

    case MEM_CMD_GET_INFO:
      {
        uint8_t memId = p->data[1];
        createInfoResponse(p, memId);
        crtpSendPacket(p);
      }
      break;

    default:
      // Do nothing
      break;
  }
}

static void createNbrResponse(CRTPPacket* p) {
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_NBR;
  p->data[1] = nbrOwMems + OW_FIRST_ID;
}

static void createInfoResponse(CRTPPacket* p, uint8_t memId) {
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_SETTINGS_CH);
  p->size = 2;
  p->data[0] = MEM_CMD_GET_INFO;
  p->data[1] = memId;

  // No error code if we fail, just send an empty packet back
  switch(memId) {
    case EEPROM_ID:
      createInfoResponseBody(p, MEM_TYPE_EEPROM, EEPROM_SIZE, eepromSerialNum.data);
      break;
    case LEDMEM_ID:
      createInfoResponseBody(p, MEM_TYPE_LED12, sizeof(ledringmem), noData);
      break;
    case LOCO_ID:
      createInfoResponseBody(p, MEM_TYPE_LOCO, MEM_LOCO_ANCHOR_BASE + MEM_LOCO_ANCHOR_PAGE_SIZE * LOCO_MESSAGE_NR_OF_ANCHORS, noData);
      break;
    case TRAJ_ID:
      createInfoResponseBody(p, MEM_TYPE_TRAJ, sizeof(trajectories_memory), noData);
      break;
    case LOCO2_ID:
      createInfoResponseBody(p, MEM_TYPE_LOCO2, MEM_LOCO_ANCHOR_BASE + MEM_LOCO_ANCHOR_PAGE_SIZE * 256, noData);
      break;
    case LH_ID:
      createInfoResponseBody(p, MEM_TYPE_LH, sizeof(lighthouseBaseStationsGeometry), noData);
      break;
    case TESTER_ID:
      createInfoResponseBody(p, MEM_TYPE_TESTER, MEM_TESTER_SIZE, noData);
      break;
    case USD_ID:
      createInfoResponseBody(p, MEM_TYPE_USD, usddeckFileSize(), noData);
      break;
    default:
      if (owGetinfo(memId - OW_FIRST_ID, &serialNbr)) {
        createInfoResponseBody(p, MEM_TYPE_OW, OW_MAX_SIZE, serialNbr.data);
      }
      break;
  }
}

static void createInfoResponseBody(CRTPPacket* p, uint8_t type, uint32_t memSize, const uint8_t data[8]) {
  p->data[2] = type;
  p->size += 1;

  memcpy(&p->data[3], &memSize, 4);
  p->size += 4;

  memcpy(&p->data[7], data, 8);
  p->size += 8;
}

/**
 * @brief Handle reading of data from the Crazyflie
 */
static void memReadProcess(CRTPPacket* p) {
  uint32_t memAddr;
  uint8_t status = STATUS_OK;

  MEM_DEBUG("Packet is MEM READ\n");
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_READ_CH);

  uint8_t memId = p->data[0];
  memcpy(&memAddr, &p->data[1], 4);
  uint8_t readLen = p->data[5];
  uint8_t* startOfData = &p->data[6];

  switch(memId) {
    case EEPROM_ID:
      status = handleEepromRead(memAddr, readLen, startOfData);
      break;

    case LEDMEM_ID:
      status = handleLedMemRead(memAddr, readLen, startOfData);
      break;

    case LOCO_ID:
      status = handleLocoMemRead(memAddr, readLen, startOfData);
      break;

    case TRAJ_ID:
      status = handleTrajectoryMemRead(memAddr, readLen, startOfData);
      break;

    case LOCO2_ID:
      status = handleLoco2MemRead(memAddr, readLen, startOfData);
      break;

    case LH_ID:
      status = handleLighthouseMemRead(memAddr, readLen, startOfData);
      break;

    case TESTER_ID:
      status = handleMemTesterRead(memAddr, readLen, startOfData);
      break;

    case USD_ID:
      status = handleUsdMemRead(memAddr, readLen, startOfData);
      break;

    default:
      status = handleOneWireMemRead(memId, memAddr, readLen, startOfData);
      break;
  }

#if 0
  {
    int i;
    for (i = 0; i < readLen; i++)
      consolePrintf("%X ", p->data[i+6]);

    consolePrintf("\nStatus %i\n", status);
  }
#endif

  p->data[5] = status;
  if (status == STATUS_OK) {
    p->size = 6 + readLen;
  } else {
    p->size = 6;
  }

  crtpSendPacket(p);
}

static uint8_t handleLocoMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* dest) {
  uint8_t status = EIO;

  // This message was defined for TWR and TDoA2 that supports up to 8 anchors
  // with ids 0 - 7. Adapt data to this format even if it means we have to
  // throw away some data to stay compatible with old clients.

  if (MEM_LOCO_INFO == memAddr) {
    if (1 == readLen) {
      *dest = LOCO_MESSAGE_NR_OF_ANCHORS;
      status = STATUS_OK;
    }
  } else {
    if (memAddr >= MEM_LOCO_ANCHOR_BASE && readLen == MEM_LOCO_PAGE_LEN) {
      uint32_t pageAddress = memAddr - MEM_LOCO_ANCHOR_BASE;
      if ((pageAddress % MEM_LOCO_ANCHOR_PAGE_SIZE) == 0) {
        uint32_t page = pageAddress / MEM_LOCO_ANCHOR_PAGE_SIZE;
        uint8_t anchorId = page;

        if (anchorId < LOCO_MESSAGE_NR_OF_ANCHORS) {
          point_t position;
          if (!locoDeckGetAnchorPosition(anchorId, &position)) {
            memset(&position, 0, sizeof(position));
          }

          float* destAsFloat = (float*)dest;
          destAsFloat[0] = position.x;
          destAsFloat[1] = position.y;
          destAsFloat[2] = position.z;

          bool hasBeenSet = (position.timestamp != 0);
          dest[sizeof(float) * 3] = hasBeenSet;

          status = STATUS_OK;
        }
      }
    }
  }

  return status;
}

static void buildAnchorList(const uint32_t memAddr, const uint8_t readLen, uint8_t* dest, const uint32_t pageBase_address, const uint8_t anchorCount, const uint8_t unsortedAnchorList[]) {
  for (int i = 0; i < readLen; i++) {
    int address = memAddr + i;
    int addressInPage = address - pageBase_address;
    uint8_t val = 0;

    if (addressInPage == 0) {
      val = anchorCount;
    } else {
      int anchorIndex = addressInPage - 1;
      if (anchorIndex < anchorCount) {
        val = unsortedAnchorList[anchorIndex];
      }
    }

    dest[i] = val;
  }
}

#define ANCHOR_ID_LIST_LENGTH 256
static uint8_t handleLoco2MemRead(uint32_t memAddr, uint8_t readLen, uint8_t* dest) {
  uint8_t status = EIO;
  static uint8_t unsortedAnchorList[ANCHOR_ID_LIST_LENGTH];

  if (memAddr >= MEM_LOCO2_ID_LIST && memAddr < MEM_LOCO2_ACTIVE_LIST) {
    uint8_t anchorCount = locoDeckGetAnchorIdList(unsortedAnchorList, ANCHOR_ID_LIST_LENGTH);
    buildAnchorList(memAddr, readLen, dest, MEM_LOCO2_ID_LIST, anchorCount, unsortedAnchorList);
    status = STATUS_OK;
  } else if (memAddr >= MEM_LOCO2_ACTIVE_LIST && memAddr < MEM_LOCO2_ANCHOR_BASE) {
    uint8_t anchorCount = locoDeckGetActiveAnchorIdList(unsortedAnchorList, ANCHOR_ID_LIST_LENGTH);
    buildAnchorList(memAddr, readLen, dest, MEM_LOCO2_ACTIVE_LIST, anchorCount, unsortedAnchorList);
    status = STATUS_OK;
  } else {
    if (memAddr >= MEM_LOCO2_ANCHOR_BASE) {
      uint32_t pageAddress = memAddr - MEM_LOCO2_ANCHOR_BASE;
      if ((pageAddress % MEM_LOCO2_ANCHOR_PAGE_SIZE) == 0 && MEM_LOCO2_PAGE_LEN == readLen) {
        uint32_t anchorId = pageAddress / MEM_LOCO2_ANCHOR_PAGE_SIZE;

        point_t position;
        memset(&position, 0, sizeof(position));
        locoDeckGetAnchorPosition(anchorId, &position);

        float* destAsFloat = (float*)dest;
        destAsFloat[0] = position.x;
        destAsFloat[1] = position.y;
        destAsFloat[2] = position.z;

        bool hasBeenSet = (position.timestamp != 0);
        dest[sizeof(float) * 3] = hasBeenSet;

        status = STATUS_OK;
      }
    }
  }

  return status;
}

/**
 * @brief Handle writing data to the Crazyflie
 */
static void memWriteProcess(CRTPPacket* p) {
  uint32_t memAddr;
  uint8_t status = STATUS_OK;

  uint8_t memId = p->data[0];
  memcpy(&memAddr, &p->data[1], 4);
  uint8_t* startOfData = &p->data[5];

  uint8_t writeLen = p->size - 5;

  MEM_DEBUG("Packet is MEM WRITE\n");
  p->header = CRTP_HEADER(CRTP_PORT_MEM, MEM_WRITE_CH);
  // Dont' touch the first 5 bytes, they will be the same.

  switch(memId) {
    case EEPROM_ID:
      status = handleEepromWrite(memAddr, writeLen, startOfData);
      break;

    case LEDMEM_ID:
      status = handleLedMemWrite(memAddr, writeLen, startOfData);
      break;

    case TRAJ_ID:
      status = handleTrajectoryMemWrite(memAddr, writeLen, startOfData);
      break;

    case LH_ID:
      status = handleLighthouseMemWrite(memAddr, writeLen, startOfData);
      break;

    case TESTER_ID:
      status = handleMemTesterWrite(memAddr, writeLen, startOfData);
      break;

    case USD_ID:
        // Not supported, fall through
    case LOCO_ID:
        // Not supported, fall through
    case LOCO2_ID:
      // Not supported
      status = EIO;
      break;

    default:
      status = handleOneWireMemWrite(memId, memAddr, writeLen, startOfData);
      break;
  }

  p->data[5] = status;
  p->size = 6;

  crtpSendPacket(p);
}

static uint8_t handleEepromRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if (memAddr + readLen <= EEPROM_SIZE &&
      eepromReadBuffer(startOfData, memAddr, readLen)) {
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleEepromWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if (memAddr + writeLen <= EEPROM_SIZE &&
      eepromWriteBuffer(startOfData, memAddr, writeLen)) {
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleLedMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if (memAddr + readLen <= sizeof(ledringmem) &&
      memcpy(startOfData, &(ledringmem[memAddr]), readLen)) {
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleLedMemWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if ((memAddr + writeLen) <= sizeof(ledringmem)) {
    memcpy(&(ledringmem[memAddr]), startOfData, writeLen);
    MEM_DEBUG("LED write addr:%i, led:%i\n", memAddr, writeLen);
    status = STATUS_OK;
  } else {
    MEM_DEBUG("\nLED write failed! addr:%i, led:%i\n", memAddr, writeLen);
    status = EIO;
  }

  return status;
}

static uint8_t handleTrajectoryMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if (memAddr + readLen <= sizeof(trajectories_memory) &&
      memcpy(startOfData, &(trajectories_memory[memAddr]), readLen)) {
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleTrajectoryMemWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if ((memAddr + writeLen) <= sizeof(trajectories_memory)) {
    memcpy(&(trajectories_memory[memAddr]), startOfData, writeLen);
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleLighthouseMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if (memAddr + readLen <= sizeof(lighthouseBaseStationsGeometry)) {
    uint8_t* start = (uint8_t*)lighthouseBaseStationsGeometry;
    memcpy(startOfData, start + memAddr, readLen);
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleLighthouseMemWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if ((memAddr + writeLen) <= sizeof(lighthouseBaseStationsGeometry)) {
    uint8_t* start = (uint8_t*)lighthouseBaseStationsGeometry;
    memcpy(start + memAddr, startOfData, writeLen);

    lightHousePositionGeometryDataUpdated();

    status = STATUS_OK;
  }

  return status;
}

/**
 * @brief The memory tester is used to verify the functionality of the memory sub system.
 * It supports "virtual" read and writes that are used by a test script to
 * check that a client (for instance the python lib) is working as expected.
 *
 * When reading data from the tester, it simply fills up a buffer with known data so that the
 * client can examine the data and verify that buffers have been correctly assembled.
 *
 * @param memAddr - the virtual address to read from
 * @param readLen - nr of bytes to read
 * @param startOfData - address to write result to
 * @return Always returns STATUS_OK
 */
static uint8_t handleMemTesterRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData) {
  for (int i = 0; i < readLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t data = addr & 0xff;
    startOfData[i] = data;
  }

  return STATUS_OK;
}

/**
 * @brief When writing data to the tester, the tester verifies that the received data
 * contains the expected values.
 *
 * @param memAddr - the virtual address to write to
 * @param writeLen - nr of bytes to write
 * @param startOfData - pointer to the data in the packet that is provided by the client
 * @return Always returns STATUS_OK
 */
static uint8_t handleMemTesterWrite(uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData) {
  if (memTesterWriteReset) {
    memTesterWriteReset = 0;
    memTesterWriteErrorCount = 0;
  }

  for (int i = 0; i < writeLen; i++) {
    uint32_t addr = memAddr + i;
    uint8_t expectedData = addr & 0xff;
    uint8_t actualData = startOfData[i];
    if (actualData != expectedData) {
      // Log first error
      if (memTesterWriteErrorCount == 0) {
        DEBUG_PRINT("Verification failed: expected: %d, actual: %d, addr: %lu\n", expectedData, actualData, addr);
      }

      memTesterWriteErrorCount++;
      break;
    }
  }

  return STATUS_OK;
}

static uint8_t handleUsdMemRead(uint32_t memAddr, uint8_t readLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  if (memAddr + readLen <= usddeckFileSize() &&
      usddeckRead(memAddr, startOfData, readLen)) {
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleOneWireMemRead(uint8_t memId, uint32_t memAddr, uint8_t readLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  uint8_t selectMem = memId - OW_FIRST_ID;
  if (memAddr + readLen <= OW_MAX_SIZE &&
      owRead(selectMem, memAddr, readLen, startOfData)) {
    status = STATUS_OK;
  }

  return status;
}

static uint8_t handleOneWireMemWrite(uint8_t memId, uint32_t memAddr, uint8_t writeLen, uint8_t* startOfData) {
  uint8_t status = EIO;

  uint8_t selectMem = memId - OW_FIRST_ID;
  if (memAddr + writeLen <= OW_MAX_SIZE &&
      owWrite(selectMem, memAddr, writeLen, startOfData)) {
    status = STATUS_OK;
  }

  return status;
}

PARAM_GROUP_START(memTst)
  PARAM_ADD(PARAM_UINT8, resetW, &memTesterWriteReset)
PARAM_GROUP_STOP(memTst)

LOG_GROUP_START(memTst)
  LOG_ADD(LOG_UINT32, errCntW, &memTesterWriteErrorCount)
LOG_GROUP_STOP(memTst)

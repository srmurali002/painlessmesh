//
//  painlessMeshComm.cpp
//
//
//  Created by Bill Gray on 7/26/16.
//
//

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SimpleList.h>

#include "painlessMesh.h"

extern painlessMesh* staticThis;

// communications functions
//***********************************************************************
bool ICACHE_FLASH_ATTR painlessMesh::sendMessage(meshConnectionType *conn, uint32_t destId, uint32_t fromId, meshPackageType type, String &msg, bool priority) {
    debugMsg(COMMUNICATION, "sendMessage(conn): conn-nodeId=%u destId=%u type=%d msg=%s\n",
             conn->nodeId, destId, (uint8_t)type, msg.c_str());

    switch (type) {
      case NODE_SYNC_REQUEST:
      case NODE_SYNC_REPLY:
      {
        DynamicJsonBuffer jsonBuffer;
        JsonArray& subs = jsonBuffer.parseArray(msg);
        if (!subs.success()) {
          debugMsg(GENERAL, "sendMessage(conn): subs = jsonBuffer.parseArray( msg ) failed!");
        }
        msg = "";
        subs.printTo(msg);
        break;
      }
      default:
        msg = msg;
        break;
    }

    // Calculate slices
    uint32_t slices = 0;
    float temp_division = msg.length() / MAX_MESSAGE_SIZE; // 1537 / 1000 = 1.537
    slices = (int)temp_division;
    if(slices > 0 && msg.length() % MAX_MESSAGE_SIZE == 0) {
      slices--;
    }

    // Abort if there are more slices than #defined
    if(slices >= MAX_MESSAGE_BUNDLE_SIZE) {
      debugMsg(ERROR, "sendMessage(conn): MAX_MESSAGE_BUNDLE_SIZE reached, Message Length: %d, Slices: %d, FreeMem: %d\n", msg.length(), slices, ESP.getFreeHeap());
      return false;
    }

    debugMsg(COMMUNICATION, "sendMessage(conn): slices=%d\n", slices);

    // Generate random packageID
    uint32_t packageID = random(0, 2147483647);

    bool failed = false;
    uint32_t startAddr = 0;
    uint32_t endAddr = MAX_MESSAGE_SIZE;
    for(int i = 0; i <= slices; i++){
      String sliced = "";
      if(i == slices) {
        sliced = msg.substring(startAddr, msg.length());
      } else {
        sliced = msg.substring(startAddr, endAddr);
        startAddr += MAX_MESSAGE_SIZE;
        endAddr += MAX_MESSAGE_SIZE;
      }

      debugMsg(COMMUNICATION, "sendMessage(conn): slice=%s\n", sliced.c_str());
      debugMsg(COMMUNICATION, "sendMessage(conn): slice.length()=%d\n", sliced.length());

      // Send the package
      String package = buildMeshPackage(destId, fromId, type, slices, i, packageID, sliced);

      while(conn->sendQueue.size() > 0) { delay(1); } // Here I am using delay because yield() causes issues...
      if(!sendPackage(conn, package, priority)) failed = true;
    }

    if(failed) return false;
    return true;
}

//***********************************************************************
bool ICACHE_FLASH_ATTR painlessMesh::sendMessage(uint32_t destId, uint32_t fromId, meshPackageType type, String &msg, bool priority) {
    debugMsg(COMMUNICATION, "In sendMessage(destId): destId=%u type=%d, msg=%s\n",
             destId, type, msg.c_str());

    meshConnectionType *conn = findConnection(destId);
    if (conn) {
        return sendMessage(conn, destId, fromId, type, msg, priority);
    } else {
        debugMsg(ERROR, "In sendMessage(destId): findConnection( %u ) failed\n", destId);
        return false;
    }
}


//***********************************************************************
bool ICACHE_FLASH_ATTR painlessMesh::broadcastMessage(uint32_t from,
                                                      meshPackageType type,
                                                      String &msg,
                                                      meshConnectionType *exclude) {

    // send a message to every node on the mesh
    bool errCode = false;

    if (exclude != NULL)
        debugMsg(COMMUNICATION, "broadcastMessage(): from=%u type=%d, msg=%s exclude=%u\n",
                 from, type, msg.c_str(), exclude->nodeId);
    else
        debugMsg(COMMUNICATION, "broadcastMessage(): from=%u type=%d, msg=%s exclude=NULL\n",
                 from, type, msg.c_str());

    SimpleList<meshConnectionType>::iterator connection = _connections.begin();
    if (_connections.size() > 0)
        errCode = true; // Assume true if at least one connections
    while (connection != _connections.end()) {
        if (connection != exclude) {
            if (!sendMessage(connection, connection->nodeId, from, type, msg))
                errCode = false; // If any error return 0
        }
        connection++;
    }
    return errCode;
}

//***********************************************************************
bool ICACHE_FLASH_ATTR painlessMesh::sendPackage(meshConnectionType *connection, String &package, bool priority) {
    debugMsg(COMMUNICATION, "Sending to %u-->%s<--\n", connection->nodeId, package.c_str());

    if (package.length() > 1400) {
        debugMsg(ERROR, "sendPackage(): err package too long length=%d\n", package.length());
        return false;
    }

    if (connection) { // Protect against null pointer
        if (connection->sendReady == true) {
            sint8 errCode = espconn_send(connection->esp_conn, (uint8*)package.c_str(), package.length());
            connection->sendReady = false;

            if (errCode == 0) {
                debugMsg(COMMUNICATION, "sendPackage(): Package sent -> %s\n", package.c_str());
                return true;
            } else {
                debugMsg(ERROR, "sendPackage(): espconn_send Failed err=%d\n", errCode);
                return false;
            }
        } else {
            if (ESP.getFreeHeap() >= MIN_FREE_MEMORY) { // If memory heap is enough, queue the message
                if (priority) {
                    connection->sendQueue.push_front(package);
                    debugMsg(COMMUNICATION, "sendPackage(): Package sent to queue beginning -> %d , FreeMem: %d\n", connection->sendQueue.size(), ESP.getFreeHeap());
                } else {
                    if (connection->sendQueue.size() < MAX_MESSAGE_QUEUE) {
                        connection->sendQueue.push_back(package);
                        debugMsg(COMMUNICATION, "sendPackage(): Package sent to queue end -> %d , FreeMem: %d\n", connection->sendQueue.size(), ESP.getFreeHeap());
                    } else {
                        debugMsg(ERROR, "sendPackage(): Message queue full -> %d , FreeMem: %d\n", connection->sendQueue.size(), ESP.getFreeHeap());
                        return false;
                    }
                }
                return true;
            } else {
                connection->sendQueue.clear(); // Discard all messages if free memory is low
                debugMsg(DEBUG, "sendPackage(): Memory low, all messages were discarded\n");
                return false;
            }
        }
    } else {
        return false;
    }
}

//***********************************************************************
String ICACHE_FLASH_ATTR painlessMesh::buildMeshPackage(uint32_t destId, uint32_t fromId, meshPackageType type, uint32_t slices, uint32_t sliceNum, uint32_t packageID, String &msg) {
    debugMsg(GENERAL, "In buildMeshPackage(): msg=%s\n", msg.c_str());

    DynamicJsonBuffer jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["dest"] = destId;
    //root["from"] = _nodeId;
    root["from"] = fromId;
    root["type"] = (uint8_t)type;
    root["slices"] = (uint8_t)slices;
    root["sliceNum"] = (uint8_t)sliceNum;
    root["packageID"] = packageID;
    root["timestamp"] = staticThis->getNodeTime();

    switch (type) {
      case NODE_SYNC_REQUEST:
      case NODE_SYNC_REPLY:
      {
        root["subs"] = msg;
        break;
      }
      case TIME_SYNC:
        root["msg"] = msg;
        break;
      default:
        root["msg"] = msg;
        break;
    }

    String ret;
    root.printTo(ret);

    debugMsg(COMMUNICATION, "sendMessage(conn): package=%s\n", ret.c_str());

    return ret;
}

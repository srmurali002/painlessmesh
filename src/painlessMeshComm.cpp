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
bool ICACHE_FLASH_ATTR painlessMesh::sendMessage( meshConnectionType *conn, uint32_t destId, meshPackageType type, String &msg ) {
    debugMsg( COMMUNICATION, "sendMessage(conn): conn-nodeId=%d destId=%d type=%d msg=%s\n",
                   conn->nodeId, destId, (uint8_t)type, msg.c_str());
    
    String package = buildMeshPackage( destId, type, msg );
    
    return sendPackage( conn, package );
}

//***********************************************************************
bool ICACHE_FLASH_ATTR painlessMesh::sendMessage( uint32_t destId, meshPackageType type, String &msg ) {
    debugMsg( COMMUNICATION, "In sendMessage(destId): destId=%d type=%d, msg=%s\n",
                   destId, type, msg.c_str());
 
    meshConnectionType *conn = findConnection( destId );
    if ( conn != NULL ) {
        return sendMessage( conn, destId, type, msg );
    }
    else {
        debugMsg( ERROR, "In sendMessage(destId): findConnection( destId ) failed\n");
        return false;
    }
}


//***********************************************************************
bool ICACHE_FLASH_ATTR painlessMesh::broadcastMessage(uint32_t from,
                                meshPackageType type,
                                String &msg,
                                meshConnectionType *exclude ) {
    
    // send a message to every node on the mesh
    
    if ( exclude != NULL )
        debugMsg( COMMUNICATION, "broadcastMessage(): from=%d type=%d, msg=%s exclude=%d\n",
                   from, type, msg.c_str(), exclude->nodeId);
    else
        debugMsg( COMMUNICATION, "broadcastMessage(): from=%d type=%d, msg=%s exclude=NULL\n",
                   from, type, msg.c_str());
    
    SimpleList<meshConnectionType>::iterator connection = _connections.begin();
    while ( connection != _connections.end() ) {
        if ( connection != exclude ) {
            sendMessage( connection, connection->nodeId, type, msg );
        }
        connection++;
    }
    return true; // hmmm... ought to be smarter than this!
}

//***********************************************************************
bool ICACHE_FLASH_ATTR painlessMesh::sendPackage( meshConnectionType *connection, String &package ) {
    debugMsg( COMMUNICATION, "Sending to %d-->%s<--\n", connection->nodeId, package.c_str() );
    
    if ( package.length() > 1400 )
        debugMsg( ERROR, "sendPackage(): err package too long length=%d\n", package.length());
    
    if ( connection->sendReady == true ) {
        sint8 errCode = espconn_send( connection->esp_conn, (uint8*)package.c_str(), package.length() );
        connection->sendReady = false;
        
        if ( errCode == 0 ) {
            return true;
        }
        else {
            debugMsg( ERROR, "sendPackage(): espconn_send Failed err=%d\n", errCode );
            return false;
        }
    }
    else {
        connection->sendQueue.push_back( package );
    }
}

//***********************************************************************
String ICACHE_FLASH_ATTR painlessMesh::buildMeshPackage( uint32_t destId, meshPackageType type, String &msg ) {
    debugMsg( GENERAL, "In buildMeshPackage(): msg=%s\n", msg.c_str() );
    
    DynamicJsonBuffer jsonBuffer( JSON_BUFSIZE );
    JsonObject& root = jsonBuffer.createObject();
    root["dest"] = destId;
    root["from"] = _nodeId;
    root["type"] = (uint8_t)type;
    root["timestamp"] = staticThis->getNodeTime();
    
    switch( type ) {
        case NODE_SYNC_REQUEST:
        case NODE_SYNC_REPLY:
        {
            JsonArray& subs = jsonBuffer.parseArray( msg );
            if ( !subs.success() ) {
                debugMsg( GENERAL, "buildMeshPackage(): subs = jsonBuffer.parseArray( msg ) failed!");
            }
            root["subs"] = subs;
            break;
        }
        case TIME_SYNC:
            root["msg"] = jsonBuffer.parseObject( msg );
            break;
        default:
            root["msg"] = msg;
    }
    
    String ret;
    root.printTo( ret );
    return ret;
}
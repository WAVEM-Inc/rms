'''
/** 
 *  Copyright (c) 2021 KT Corp. All rights reserved.
 *
 *  This is a proprietary software of KT corp, and you may not use this file except in
 *  compliance with license agreement with KT corp. Any redistribution or use of this
 *  software, with or without modification shall be strictly prohibited without prior written
 *  approval of KT corp, and the copyright notice above does not evidence any actual or
 *  intended publication of such software.
 *
 */
 '''
import os, sys
import time
import ctypes

if sys.version_info >= (3, 0) :
	from .libs import IotmakersStdDevicePy3 as client
else :
    from .libs import IotmakersStdDevicePy2 as client

class IoTMakersDeviceClient:

	def __init__(self):
		
		self.hndl = -1;

	def ImInit(self, logLevel=1):
		self.logLevel = logLevel;
		self.hndl = client.ImInit(self.logLevel);
		return self.hndl;

	def ImRelease(self):
		client.ImRelease(self.hndl);

	def ImConnectTo(self, serverIp, serverPort):
		return client.ImConnectTo(self.hndl, serverIp, serverPort);

	def ImDisconnect(self):
		return client.ImDisconnect(self.hndl);

	def ImTurnCircuitBreakerOff(self):
		return client.ImTurnCircuitBreakerOff(self.hndl);

	def ImTurnResponseWaitOff(self):
		return client.ImTurnResponseWaitOff(self.hndl);

	def ImAuthDevice(self, devId, devPw, devGw):
		return client.ImAuthDevice(self.hndl, devId, devPw, devGw);

	def ImPoll(self):
		return client.ImPoll(self.hndl);

	def ImMSleep(self, sec):
		#return client.ImMSleep(self.hndl, msec);
		time.sleep(sec/1000)


	def ImResourceSendNotification(self, resource_id, properties_in_json):
		return client.ImResourceSendNotification(self.hndl, resource_id, properties_in_json);
	
	def ImResourceNotificationInit(self):
		return client.ImResourceNotificationInit(self.hndl);

	def ImResourceNotificationAppendResource(self, resource_id, properties_in_json):
		return client.ImResourceNotificationAppendResource(self.hndl, resource_id, properties_in_json);

	def ImResourceNotificationSend(self):
		return client.ImResourceNotificationSend(self.hndl);


	def ImSetControlCallBackHandler(self, OnResourceSetRequestHandler=None, OnResourceRetrieveOneRequestHandler=None, OnResourceRetrieveAllRequestHandler=None):
		client.ImSetOnResourceSetRequestHandler(self.hndl, OnResourceSetRequestHandler);
		client.ImSetOnResourceRetrieveOneRequestHandler(self.hndl, OnResourceRetrieveOneRequestHandler);
		client.ImSetOnResourceRetrieveAllRequestHandler(self.hndl, OnResourceRetrieveAllRequestHandler);


	def ImResourceRetrieveSetResource(self, pktBody, resource_id, properties_in_json):
		return client.ImResourceRetrieveSetResource(pktBody, resource_id, properties_in_json);

	def ImResourceRetrieveAppendResource(self, pktBody, resource_id, properties_in_json):
		return client.ImResourceRetrieveAppendResource(pktBody, resource_id, properties_in_json);




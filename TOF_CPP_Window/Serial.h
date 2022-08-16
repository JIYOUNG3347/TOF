#pragma once
/*
* Permission to use, copy, modify, distributeand sell this software
*and its documentation for any purpose is hereby granted without fee
* provided that the above copyright notice appear in all copiesand
* that both that copyright noticeand this permission notice appear
* in supporting documentation.
* It is provided "as is" without express or implied warranty.
*
*/

#ifndef SERIALPORT___H
#define SERIALPORT___H

#include <windows.h>

class TSerialPort
{
private:
    HANDLE m_portHandle;
    HANDLE m_workingThread;
    DWORD  m_workingThreadId;

    int    m_timeoutMilliSeconds;
    int    m_maxPacketLength;

    void (*m_OnDataReceivedHandler)(const unsigned char* pData, int dataLength);
    void (*m_OnDataSentHandler)(void);

    CRITICAL_SECTION m_criticalSectionRead;
    CRITICAL_SECTION m_criticalSectionWrite;

    int __ReadBuffer(unsigned char* pData, int dataLength, int timeOutMS = -1);
    int __WriteBuffer(const char* pData, int dataLength);


public:
    TSerialPort();
    ~TSerialPort();

    int GetMaxTimeout();
    void* GetDataReceivedHandler();
    void* GetDataSentHandler();

    bool Open(int comPortNumber, int baudRate, int timeoutMS = 1000);

    bool OpenAsync(int comPortNumber, int baudRate,
        void (*OnDataReceivedHandler)(const unsigned char* pData, int dataLength),
        void (*OnDataSentHandler)(void),
        int timeoutMS = 100
    );

    void Close();
    bool IsOpen();

    int ReadBuffer(unsigned char* pData, int dataLength, int timeOutMS = -1);
    int WriteBuffer(const char* pData, int dataLength);

    int ReadLine(char* pLine, int maxBufferSize, int timeOutMS = -1);
    int WriteLine(char* pLine, bool addCRatEnd = true);


};

DWORD WINAPI SerialPort_WaitForData(LPVOID lpParam);


#endif
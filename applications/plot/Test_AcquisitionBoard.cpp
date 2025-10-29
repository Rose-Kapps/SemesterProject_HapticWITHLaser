// Test_AcquisitionBoard.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "cbw.h"
#include <cmath>
#include <algorithm>
#include "Data_process.h"
#include "chai3d.h"

using namespace chai3d;

// GLOBAL VARIABLES

double voltageLevel;



// FUNCTION DECLARATION

// this function contains the laser sensor aquisition loop
void updateSensor(void);



int main()
{
    std::cout << "Hello World! v2\n" << endl;
    while (true) {
        std::cout << "Sensor Value: ";
        updateSensor();
        std::cout << voltageLevel << endl;
    }
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file




// FUNCTION DEFINITION

void updateSensor(void)
{
    //--------------------------------------------------------------------------
    // INIT I/O BOARD
    //--------------------------------------------------------------------------

    int Row, Col;
    int BoardNum = 0;
    int ULStat = 0;
    int LowChan = 0;
    int HighChan = 0;
    int Range = UNI10VOLTS;
    short Status = 0;
    long CurCount;
    long CurIndex;
    int Count = 100;
    long Rate = 10000;
    unsigned Options;
    HANDLE MemHandle = 0;
    WORD* ADData;
    DWORD* ADData32;
    float    RevLevel = (float)CURRENTREVNUM;
    BOOL HighResAD = FALSE;
    int  ADRes;

    // Declare UL Revision Level
    ULStat = cbDeclareRevision(&RevLevel);

    // Initiate error handling
    //     Parameters:
    //         PRINTALL :all warnings and errors encountered will be printed
    //         DONTSTOP :program will continue even if error occurs.
    //                  Note that STOPALL and STOPFATAL are only effective in
    //                  Windows applications, not Console applications.
    cbErrHandling(PRINTALL, DONTSTOP);

    // Get the resolution of A/D
    cbGetConfig(BOARDINFO, BoardNum, 0, BIADRES, &ADRes);

    // If ADRes is equal to zero, exit as card has not acquisition card has not been detected
    if (ADRes == 0)
    {
        return;
    }

    // check If the resolution of A/D is higher than 16 bit.
    //    If it is, then the A/D is high resolution.
    if (ADRes > 16)
        HighResAD = TRUE;

    //  set aside memory to hold data
    if (HighResAD)
    {
        MemHandle = cbWinBufAlloc32(Count);
        ADData32 = (DWORD*)MemHandle;
    }
    else
    {
        MemHandle = cbWinBufAlloc(Count);
        ADData = (WORD*)MemHandle;
    }

    if (!MemHandle)
    {
        printf("\nout of memory\n");
        exit(1);
    }

    WORD DataValue;
    ULStat = cbFromEngUnits(BoardNum, Range, 0.0, &DataValue);
    ULStat = cbAOut(BoardNum, 0, Range, DataValue);

    // Collect the values with cbAInScan() in BACKGROUND mode
    //     Parameters:
    //          BoardNum    :the number used by CB.CFG to describe this board
    //          LowChan     :low channel of the scan
    //          HighChan    :high channel of the scan
    //          Count       :the total number of A/D samples to collect
    //          Rate        :sample rate in samples per second
    //          Gain        :the gain for the board
    //          ADData[]    :the array for the collected data values
    //          Options     :data collection options
    Options = NOCONVERTDATA + FOREGROUND; // BACKGROUND;


    // start data scanning operation
    ULStat = cbAInScan(BoardNum, LowChan, HighChan, Count, &Rate, Range, MemHandle, Options);

    // wait for data aquisition to complete
    Status = RUNNING;
    while (Status == RUNNING)
    {
        // check the status of the current background operation
        // parameters:
        //     BoardNum  :the number used by CB.CFG to describe this board
        //     Status    :current status of the operation (IDLE or RUNNING)
        //     CurCount  :current number of samples collected
        //     CurIndex  :index to the last data value transferred
        //     FunctionType: A/D operation (AIFUNCTIOM)
        ULStat = cbGetStatus(BoardNum, &Status, &CurCount, &CurIndex, AIFUNCTION);
    }

    // process data
    for (int i = 0; i < Count; i++)
    {
        // get next data value
        WORD dataValue = 0;

        if (HighResAD)
        {
            dataValue = ADData32[i];
        }
        else
        {
            dataValue = ADData[i];
        }

        //Apply a gaussian filter to smoothen sensor values
        static GaussianFilter filter(9, 1.03);
        voltageLevel = filter.applyFilter(voltageLevel);

        // convert data value to a sensor voltage level
        voltageLevel = round(100 * cClamp(5.0 * (((double)(dataValue)-2048.0) / 964.0), 0.0, 5.0)) / 100;



        //// update frequency counter
        //freqCounterSensor.signal(Count);
    }

    // the BACKGROUND operation must be explicitly stopped
    //     Parameters:
    //          BoardNum    :the number used by CB.CFG to describe this board
    //          FunctionType: A/D operation (AIFUNCTIOM)
    ULStat = cbStopBackground(BoardNum, AIFUNCTION);

    cbWinBufFree(MemHandle);

}
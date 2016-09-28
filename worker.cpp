#include "worker.h"

Worker::Worker(QObject *parent) :    QObject(parent)
{
    MFRC522 *reader;
    QTextStream out(stdout);

    out << "Starting RFID reader v.0.1.4 - SPI...." << endl;
    reader = new MFRC522(NULL);
    reader->PCD_Init();
    buz = new BuzzThread(this);
    buz->start();

    while (true) {
        // Look for new cards
        if ( ! reader->PICC_IsNewCardPresent()) continue;
        // Select one of the cards
        if ( ! reader->PICC_ReadCardSerial())  continue;
        //Show UID on stdout
        QString uid;
        for(int i=0 ; i<reader->uid.size ; i++)
        {
            uid += ( QString::number(reader->uid.uidByte[i], 16) + " ");
        }

        out << "UID tag :" << uid  << endl;
        buz->buzz(100);
        sleep(1);
    }//endless loop
}



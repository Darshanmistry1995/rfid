#include "worker.h"

Worker::Worker(QObject *parent) :    QObject(parent)
{
    MFRC522 *reader;
    QTextStream out(stdout);

    out << "Starting RFID reader v.0.1.1 - SPI...." << endl;
    reader = new MFRC522(NULL);

    reader->PCD_Init();
/*
    fs.open("/sys/class/gpio/gpio115/value", std::fstream::out);
    fs << "0";
    fs.close();
*/

    while (true) {
        // Look for new cards
        if ( ! reader->PICC_IsNewCardPresent()) continue;
        // Select one of the cards
/*
        fs.open("/sys/class/gpio/gpio115/value", std::fstream::out);
        fs << "1";
        fs.close();
*/
        if ( ! reader->PICC_ReadCardSerial())  continue;

        //Show UID on stdout
        QString uid;
        for(int i=0 ; i<reader->uid.size ; i++)
        {
            uid += ( QString::number(reader->uid.uidByte[i], 16) + " ");
        }

        out << "UID tag :" << uid  << endl;
        sleep(1);
    }//endless loop
}



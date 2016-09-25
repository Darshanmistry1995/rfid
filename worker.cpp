#include "worker.h"

Worker::Worker(QObject *parent) :    QObject(parent)
{
    MFRC522 *reader;
    QTextStream out(stdout);

    out << "Starting RFID reader v.0.0.10 ...." << endl;
    reader = new MFRC522(NULL, "/dev/ttyO2");

    reader->PCD_Reset();
    sleep(1);

    reader->PCD_Init();

    out << "ComIrqReg[4]=" << endl;
    byte reg = reader->PCD_ReadRegister(0x04);

    while (true) {

        out << endl;

         sleep(5);
        // Look for new cards
        if ( ! reader->PICC_IsNewCardPresent()) continue;
        out << "PICC_IsNewCardPresent()=true" << endl;

        // Select one of the cards
      //  if ( ! reader->PICC_ReadCardSerial())  continue;
      //  out<< "PICC_ReadCardSerial()=true" <<endl;
        reader->PICC_ReadCardSerial();


        //Show UID on serial monitor
        QString uid;
        out << "uid.size=" << QString::number(reader->uid.size) << endl;
        for(int i=0 ; i<reader->uid.size ; i++)
        {
            uid += ( QString::number(reader->uid.uidByte[i], 16) + " ");
        }

        out << "UID tag :" << uid  << endl;


    }//endless loop
}

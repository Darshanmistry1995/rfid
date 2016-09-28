#include "worker.h"

QSqlDatabase dbase;

Worker::Worker(QObject *parent) :    QObject(parent)
{
    MFRC522 *reader;
    QTextStream out(stdout);

    out << "Starting RFID reader v.0.1.8 - SPI...." << endl;
    reader = new MFRC522(NULL);
    reader->PCD_Init();

    buz = new BuzzThread(this);
    buz->start();

   // manager = new QNetworkAccessManager(this);
    connect(&manager, SIGNAL(finished(QNetworkReply*)), this, SLOT(replyFinished(QNetworkReply*)));

    //Open database
    QFile cdbFile("data.db");
    if(!cdbFile.exists()) out << "ERR: database file not found" <<  endl;
    else
    {   dbase = QSqlDatabase::addDatabase("QSQLITE"); //QSQLITE е за версия 3 и нагоре, QSQLITE2 e за версия 2
        dbase.setDatabaseName("data.db");
        if(!dbase.open()) out << "ERR: can't open database" << endl;
    }

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

//--------------------------------------------------------
        QString strUrl="http://www.etherino.net/rfid-test.php?tag=FA-12-34-56&posid=STOL1&timestamp=28/09/2016%2020:50:39";
        QNetworkReply *rep= manager.get(QNetworkRequest(QUrl(strUrl)));
        QTime ttt;
        ttt.start();
        while(1)
        {
            if(rep->isFinished())
            {
                out << "FIN!" << endl;
                break;
            }
            if(ttt.elapsed()>5000)
            {
                out << " TIME" << endl;
                break;
            }
            sleep(5);
        }
//--------------------------------------------------------------------
        QSqlQuery qry;
        bool ok;
        //                         Timestamp                tag
        //INSERT INTO queue VALUES ('2012-11-24 10:10:10', '11 22 33 44');
        QString str = QString("INSERT INTO queue VALUES ('%1 %2','%3');")
                      .arg(QDate::currentDate().toString("yyyy-MM-dd")) //1
                      .arg(QTime::currentTime().toString("hh:mm:ss"))   //2
                      .arg(uid);  //3
        qry.prepare(str);
        ok = qry.exec();
        if(!ok) out << "ERR: can't write to database queue table" << endl;

        sleep(1);
    }//endless loop
}


void Worker::replyFinished(QNetworkReply * reply)
{
    QTextStream out(stdout);
    out << endl << "2.REPLY:" << endl;

    //QByteArray repData = reply->readAll();
    //out << QString(repData);

    reply->deleteLater();
}

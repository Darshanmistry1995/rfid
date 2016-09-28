#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QTextStream>
#include <QFile>

#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QUrl>
#include <QSslConfiguration>

#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>

#include "mfrc522.h"
#include "buzThread.h"

class Worker : public QObject
{
    Q_OBJECT
public:
    explicit Worker(QObject *parent = 0);


private:
    BuzzThread  *buz;
    //За POST изпращане на данни
    QNetworkAccessManager manager;
    QNetworkRequest request;


signals:
    
public slots:
    void replyFinished(QNetworkReply *reply); //get reply from server
};

#endif // WORKER_H

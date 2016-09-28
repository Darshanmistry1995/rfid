#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QTextStream>
#include "mfrc522.h"
#include "buzThread.h"

class Worker : public QObject
{
    Q_OBJECT
public:
    explicit Worker(QObject *parent = 0);

private:
    BuzzThread  *buz;
signals:
    
public slots:
    
};

#endif // WORKER_H

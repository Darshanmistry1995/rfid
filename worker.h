#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QTextStream>
#include "mfrc522.h"

class Worker : public QObject
{
    Q_OBJECT
public:
    explicit Worker(QObject *parent = 0);
    
signals:
    
public slots:
    
};

#endif // WORKER_H

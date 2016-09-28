#ifndef BUZTHREAD_H
#define BUZTHREAD_H

#include <QThread>
#include "definitions.h"
#include <fstream>
#include <stdio.h>

class BuzzThread : public QThread
{
    Q_OBJECT
public:
    explicit BuzzThread(QObject *parent = 0);
    
    void run();
    void buzz(int ms); //Assigns new value to counter

private:
    int counter; // Counts down to zero every 10ms. When counter==0 the buzzer is turned off;
    bool buzzing;
    std::fstream fs;
signals:
    
public slots:
    

};

#endif // BUZTHREAD_H

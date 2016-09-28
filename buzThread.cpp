#include "buzThread.h"
/**
 * @brief buzThread::buzThread
 * @param parent
 *
 * T
 */
BuzzThread::BuzzThread(QObject *parent) :   QThread(parent)
{
    fs.open(GPIO_BUZZER_VALUE, std::fstream::out);
    fs << "0";
    fs.close();
    buzzing=false;
}


void BuzzThread::run()
{
    while(1)
    {
        if(counter>1000) counter=1000; //no buzz longer than a second.

        if(counter<=0) { // turn off the buzzer
            if(buzzing) {
                fs.open(GPIO_BUZZER_VALUE, std::fstream::out);
                fs << "0";
                fs.close();
                buzzing=false;
            }
        }
        else counter-= 10;

        usleep(10000);
    }
}

void BuzzThread::buzz(int ms)
{
    if(counter<ms) counter=ms;
    if(counter>1000) counter=1000; //no buzz longer than a second.
    if(buzzing) return;
    fs.open(GPIO_BUZZER_VALUE, std::fstream::out);
    fs << "1";
    fs.close();
    buzzing=true;
}

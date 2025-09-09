#ifndef COMMANDSTRUCT_H
#define COMMANDSTRUCT_H

#include <QObject>
#include <stdio.h>
#include <stdlib.h>
#include <QString>

class CommandStruct
{
public:
    explicit CommandStruct(QString keyword = " ", int param_len = 0);
    ~CommandStruct();

    int get_param_len() const;
    QString get_keyword() const;


private:
    QString keyword;
    int param_len;
};

#endif // COMMANDSTRUCT_H

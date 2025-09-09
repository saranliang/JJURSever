#include "CommandStruct.h"

CommandStruct::CommandStruct(QString keyword, int len)
{
    this->keyword = keyword;
    this->param_len = len;
}

int CommandStruct::get_param_len() const
{
    return this->param_len;
}

QString CommandStruct::get_keyword() const
{
    return this->keyword;
}

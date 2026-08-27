//
// Created by CaoKangqi on 2026/6/19.
//

#ifndef HORIZON_MESSAGE_CENTER_H
#define HORIZON_MESSAGE_CENTER_H

#include <stdint.h>
#include <stddef.h>

// 句柄别名，对齐第一份代码的业务层调用
typedef void Publisher_t;
typedef void Subscriber_t;

Publisher_t* PubRegister(const char *name, void *external_ptr, size_t size);
Subscriber_t* SubRegister(const char *name, size_t size);

void PubPushMessage(Publisher_t *pub_handle, const void *data);
void SubGetMessage(Subscriber_t *sub_handle, void *buffer);

void Message_Center_Init(void);

#endif //HORIZON_MESSAGE_CENTER_H
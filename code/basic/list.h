/*
 * list.h
 *
 *  Created on: 2023年11月30日
 *      Author: symct
 */

#ifndef CODE_ALGORITHM_BASIC_LIST_H_
#define CODE_ALGORITHM_BASIC_LIST_H_

#include "zf_common_headfile.h"
#include "define.h"
#include "common.h"
typedef struct Node{
    uint16 id;
    Point data;
    struct Node *next;
    double angle;
}LinkList;

LinkList* createLinkList(Point *p);
Point getNode(LinkList *head, uint16 id);
void tailInsertNode(LinkList *head, Point *p);
void deleteNode(LinkList *head, uint16 id);
void insertNode(LinkList *head, Point *p, uint16 id);
void freeLinkList(LinkList **head);
void connectList(LinkList *head1, LinkList *head2);
extern uint16 idMax;
#endif /* CODE_ALGORITHM_BASIC_LIST_H_ */

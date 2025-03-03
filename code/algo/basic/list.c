/*
 * list.c
 *
 *  Created on: 2023年11月30日
 *      Author: symct
 */


#include "list.h"
uint16 idMax = 0;
/// @brief create a new link list
/// @param p original point
/// @return LinkList*
LinkList* createLinkList(Point *p){
    LinkList *node;
    node = (LinkList *)malloc(sizeof(LinkList));
    if(node == NULL){
        return NULL;
    }
    memset(node, 0, sizeof(LinkList));
    node->data.x = p->x;
    node->data.y = p->y;
    node->next = NULL;
    node->id = 0;
    return node;
}
/// @brief get the node by id
/// @param head head of the link list
/// @param id id of the node
/// @return Point
Point getNode(LinkList *head, uint16 id){
    LinkList *ptr = head;
    while(ptr != NULL){
        if(ptr->id == id){
            return ptr->data;
        }
        ptr = ptr->next;
    }
    return (Point){0, 0};
}
/// @brief insert a node to the tail of the link list
/// @param head head of the link list
/// @param p point to be inserted
void tailInsertNode(LinkList *head, Point *p){
    LinkList *ptr = head;
    while(ptr->next != NULL){
        ptr = ptr->next;
    }
    ptr->next = (LinkList *)malloc(sizeof(LinkList));
    if(ptr->next == NULL){
        return;
    }
    ptr->next->data.x = p->x;
    ptr->next->data.y = p->y;
    ptr->next->next = NULL;
    ptr->next->id = ptr->id + 1;
    idMax = ptr->next->id;
}
/// @brief delete a node by id
/// @param head head of the link list
/// @param id id of the node
void deleteNode(LinkList *head, uint16 id){
    LinkList *ptr = head;
    while(ptr->next != NULL){
        if(ptr->next->id == id){
            if(ptr->next->next == NULL){//tail
                free(ptr->next);
                ptr->next = NULL;
                break;
            }
            LinkList *t = ptr->next;
            ptr->next = ptr->next->next;
            free(t);
            LinkList * temp = ptr->next;
            while(temp != NULL){
                temp->id--;
                temp = temp->next;
            }
            break;
        }
        ptr = ptr->next;
    }
}
/// @brief insert a node to the link list
/// @param head head of the link list
/// @param p point to be inserted
/// @param id id of the node
void insertNode(LinkList *head, Point *p, uint16 id){
    LinkList *ptr = head;
    LinkList *pre;
    while(ptr!= NULL){
        pre = ptr;

        if(ptr->next == NULL){//tail
            tailInsertNode(head, p);
            break;
        }
        if(ptr->next->id == id){
            LinkList *node = (LinkList *)malloc(sizeof(LinkList));
            if(node == NULL){
                return;
            }
            node->data.x = p->x;
            node->data.y = p->y;
            node->next = ptr->next;
            node->id = id;
            pre->next = node;
            LinkList * temp = node->next;
            while(temp != NULL){
                temp->id++;
                temp = temp->next;
            }
            break;
        }
        ptr = ptr->next;
    }
}
/// @brief release the link list
/// @param head head of the link list
void freeLinkList(LinkList **head){
    LinkList *ptr = *head;
    while(ptr != NULL){
        LinkList *temp = ptr;
        ptr = ptr->next;
        free(temp);
    }
    *head = NULL;
}
/// @brief connect two link list
/// @param head1 the first link list
/// @param head2 the second link list
void connectList(LinkList *head1, LinkList *head2){
    LinkList *ptr = head1;
    while(ptr->next != NULL){
        ptr = ptr->next;
    }
    ptr->next = head2;
}
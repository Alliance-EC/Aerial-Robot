#include "UI_Interface.h"
#define QUEUE_MAX_SIZE 100

// // 队列结构体
// typedef struct {
//     int queue[QUEUE_MAX_SIZE]; // 存储整数的数组
//     int front;                 // 队头索引
//     int rear;                  // 队尾索引
//     int count;                 // 队列中的元素数量
// } IntQueue;

// // 初始化队列
// void InitQueue(IntQueue *q)
// {
//     q->front = 0;
//     q->rear  = 0;
//     q->count = 0;
// }

// // 判断队列是否为空
// bool IsQueueEmpty(const IntQueue *q)
// {
//     return q->count == 0;
// }

// // 判断队列是否为满
// bool IsQueueFull(const IntQueue *q)
// {
//     return q->count == QUEUE_MAX_SIZE;
// }

// // 入队操作，如果队列已满则不入队
// bool EnQueue(IntQueue *q, int value)
// {
//     if (IsQueueFull(q)) {
//         return false; // 队列已满，返回false
//     }
//     q->queue[q->rear] = value;
//     q->rear           = (q->rear + 1) % QUEUE_MAX_SIZE;
//     q->count++;
//     return true;
// }

// // 出队操作
// bool DeQueue(IntQueue *q)
// {
//     if (IsQueueEmpty(q)) {
//         return false; // 队列为空，返回false
//     }
//     q->front = (q->front + 1) % QUEUE_MAX_SIZE;
//     q->count--;
//     return true;
// }

// // 查重逻辑，如果队列中存在相同的值，则不入队
// bool IsDuplicate(const IntQueue *q, int value)
// {
//     for (int i = 0; i < q->count; i++) {
//         int index = (q->front + i) % QUEUE_MAX_SIZE;
//         if (q->queue[index] == value) {
//             return true;
//         }
//     }
//     return false;
// }

// // 检查队列中的元素是否满足发送条件，并发送
// void CheckAndSend(IntQueue *q)
// {
//     if (q->count >= 7) { // 队列满7个元素时发送
//         for (int i = 0; i < 7; i++) {
//             sendData(q->queue[(q->front + i) % QUEUE_MAX_SIZE]);
//         }
//         // 发送完毕后，出队7个元素
//         for (int i = 0; i < 7; i++) {
//             DeQueue(q);
//         }
//     }
// }

void Is_change(Graph_Data_t *graph_now, Graph_Data_t *graph_last)
{
    //graph_now->active_flag = memcmp(graph_now, graph_last, sizeof(Graph_Data_t));
}

// void MY_Queue_Add(){}

// void MY_High_Refresh(){
//     //初始化队列？是否需要
//     while (IsQueueEmpty()!=0){
//     //发送
//     if (q->count == 7) {
//         sendData(7);
//         DeQueue(q, 7);
//         return;
//     }
//     if (q->count >= 5) {
//         sendData(5);
//         DeQueue(q, 5);
//         return;
//     }
//     // 然后尝试发送2个包
//     if (q->count >= 2) {
//         sendData(2);
//         DeQueue(q, 2);
//         return;
//     }
//     // 最后尝试发送1个包
//     if (q->count >= 1) {
//         sendData(1);
//         DeQueue(q, 1);
//     }
//     }
// }
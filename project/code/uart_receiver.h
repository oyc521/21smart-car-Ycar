#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H

#include <rtthread.h>
#include <stdint.h>

/* ���λ�������С������������һ֡���ݳ��� */
#define UART_RX_BUF_SIZE    2048

/* ������������С������������֡���� */
#define PARSE_BUF_SIZE      2048

/* ���λ������ṹ�� */
typedef struct {
    uint8_t buffer[UART_RX_BUF_SIZE];
    volatile uint16_t head;     /* д���������ж��и��£� */
    volatile uint16_t tail;     /* ��ȡ�������߳��и��£� */
} ring_buffer_t;

/* ȫ�ֻ��λ�����ʵ�� */
extern ring_buffer_t g_uart_rb;

/* ��ͼ���±�־���ɽ����߳����ã����̲߳�ѯ�� */
extern uint8_t g_map_updated;
/* ��Ҫ�����ͼ��־���ɻ�Ͽ��������ã����̷߳������� */
extern volatile uint8_t need_map_update;

/* ��ʼ�����ڽ��գ����� UART1�������жϣ� */
void uart_receive_init(void);

/* �����߳���ں��� */
void parse_uart_data_thread_entry(void *parameter);

/* �ⲿ�����滮�������� uart_receiver.c �ж��壩 */
void trigger_planning(void);

// uart_receiver.h ���������º������������ļ�ĩβ��
void get_vision_position(float *x, float *y);
uint8_t is_vision_valid(void);
void clear_vision_valid(void);

#endif
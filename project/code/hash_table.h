#ifndef HASH_TABLE_H
#define HASH_TABLE_H

#include <stdint.h>

// 哈希表条目结构
typedef struct {
    int key;
    int parent_key;
    int action;
    int pushes;
    int moves;
    uint8_t used;  // 0 未用，1 已用
} HashEntry;

// 哈希表操作函数
void hash_init(HashEntry* table, int size);
int hash_find(HashEntry* table, int size, int key);
int hash_insert(HashEntry* table, int size, int key, int parent, int action, int pushes, int moves);
int hash_find_with_used(const HashEntry* table, const uint8_t* used, int size, int key);
int hash_insert_with_used(HashEntry* table, uint8_t* used, int size, int key,
                          int parent, int action, int pushes, int moves);

#endif
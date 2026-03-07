#include "hash_table.h"
#include <string.h>

void hash_init(HashEntry* table, int size) {
    memset(table, 0, size * sizeof(HashEntry));
}

static int hash_func(int key, int size) {
    return key % size;
}

int hash_find(HashEntry* table, int size, int key) {
    int idx = hash_func(key, size);
    int start = idx;
    while (table[idx].used) {
        if (table[idx].key == key) return idx;
        idx = (idx + 1) % size;
        if (idx == start) break;
    }
    return -1;
}

int hash_insert(HashEntry* table, int size, int key, int parent, int action, int pushes, int moves) {
    int idx = hash_func(key, size);
    int start = idx;
    while (table[idx].used) {
        if (table[idx].key == key) return idx; // 已存在
        idx = (idx + 1) % size;
        if (idx == start) return -1; // 表满
    }
    table[idx].used = 1;
    table[idx].key = key;
    table[idx].parent_key = parent;
    table[idx].action = action;
    table[idx].pushes = pushes;
    table[idx].moves = moves;
    return idx;
}
int hash_find_with_used(const HashEntry* table, const uint8_t* used, int size, int key) {
    int idx = key % size;
    int start = idx;
    while (used[idx]) {
        if (table[idx].key == key) return idx;
        idx = (idx + 1) % size;
        if (idx == start) break;
    }
    return -1;
}

int hash_insert_with_used(HashEntry* table, uint8_t* used, int size, int key,
                          int parent, int action, int pushes, int moves) {
    int idx = key % size;
    int start = idx;
    while (used[idx]) {
        if (table[idx].key == key) return idx; // 已存在
        idx = (idx + 1) % size;
        if (idx == start) return -1; // 表满
    }
    used[idx] = 1;
    table[idx].key = key;
    table[idx].parent_key = parent;
    table[idx].action = action;
    table[idx].pushes = pushes;
    table[idx].moves = moves;
    return idx;
}
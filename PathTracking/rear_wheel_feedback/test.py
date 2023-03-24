#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

def print_numbers():
    for i in range(10):
        print(i)

# 创建一个线程
thread = threading.Thread(target=print_numbers)

# 启动线程
thread.start()

# 等待线程结束
thread.join()






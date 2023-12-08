import multiprocessing

def process_function(result_list, process_name):
    # 模拟进程生成数据
    data = [process_name + " data 1", process_name + " data 2", process_name + " data 3"]
    result_list.extend(data)

if __name__ == '__main__':
    manager = multiprocessing.Manager()
    result_list = manager.list()

    # 创建三个进程
    p1 = multiprocessing.Process(target=process_function, args=(result_list, "Process 1"))
    p2 = multiprocessing.Process(target=process_function, args=(result_list, "Process 2"))
    p3 = multiprocessing.Process(target=process_function, args=(result_list, "Process 3"))

    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p2.join()
    p3.join()

    # 输出合并后的数据
    print(result_list)
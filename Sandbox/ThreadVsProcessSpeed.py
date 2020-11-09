from threading import Thread
from multiprocessing import Process
from time import time
import random


def task(x, i):
    variates = []
    for j in range(1000):
        variates.append(random.normalvariate(0, 1))
    x[i] += sum(variates)
    del variates


if __name__ == '__main__':
    num_concurrent = 10
    threads = []
    num_reps = 10

    def testImplementation(thread_or_process):
        threads = []
        avg_startup_time = 0
        avg_total_time = 0
        x_list = [0]*num_concurrent
        for reps in range(num_reps):
            start = time()
            for t in range(num_concurrent):
                new_thread = thread_or_process(target=task, args=(x_list, t,))
                new_thread.start()
                threads.append(new_thread)
            avg_startup_time += time() - start
            for thread in threads:
                thread.join()

            del threads
            threads = []

            avg_total_time += time() - start
        avg_startup_time /= num_reps*num_concurrent
        avg_total_time /= num_reps*num_concurrent
        print(f'Average startup time = {avg_startup_time}\nAverage total time = {avg_total_time}')


    testImplementation(Thread)
    testImplementation(Process)

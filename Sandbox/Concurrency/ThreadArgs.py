from threading import Thread, Event


if __name__ == '__main__':
    num_concurrent = 10
    threads = []
    num_reps = 10

    def testFunc1(a, key1=True):
        print(a, key1)

    def testFunc2(a, b, key1=False, key2='0'):
        print(a, b, key1, key2)

    def testFunc5(a, b, c, d, e, key1=True, key2='1'):
        print(a, b, c, d, e, key1, key2)

    def testArguments(args1, kwargs1): #, args2, kwargs2, args3, kwargs3
        print("starting threads")
        thread1 = Thread(target=testFunc1, args=args1, kwargs=kwargs1, daemon=True, name="thread1")
        thread1.start()
        thread1.join()

    args1 = 'abba'
    kwargs1 = {'key1': False}
    testArguments(args1, kwargs1)

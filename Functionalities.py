import os
import time
import traceback
import tracemalloc
from threading import Event


pi = 3.14159265359


def sleep(time_to_sleep, stop_event):
    if not isinstance(time_to_sleep, float):
        raise TypeError(f"{time_to_sleep} is not an adequate quantity for time")
    if not isinstance(stop_event, Event):
        raise TypeError(f"{stop_event} is not an event")

    start_time = time.time()
    while not stop_event.isSet():
        now = time.time()
        if now - start_time >= time_to_sleep:
            break
        else:
            time.sleep(0.1)


def communicateError(exception, message_extra=""):
    tb = exception.__traceback__
    summary = traceback.extract_tb(tb, limit=-1)[0]

    type_exc = exception.__class__.__name__
    text = str(exception) if not message_extra else message_extra
    if len(text) > 0 and text[-1] == '.':
        text = text[:-1]
    func_name = summary.name
    file_name = os.path.split(summary.filename)[1]
    line_no = summary.lineno
    problem = summary._line

    message = f'{type_exc}({text}) in {func_name}(), file {file_name}, line {line_no}.'
    if problem and problem[0:5] != 'raise':
        message += f' Cause: {problem}'

    FAIL = '\033[91m'
    END = '\033[0m'
    print(FAIL + message + END)


def testMemoryDemand():
    tracemalloc.start(10)
    while True:
        snapshot = tracemalloc.take_snapshot()
        for i, stat in enumerate(snapshot.statistics('filename')[:4], 1):
            print(str(stat))
        time.sleep(60.0)

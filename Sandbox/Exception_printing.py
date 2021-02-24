import os
import traceback


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
    if problem[0:5] != 'raise':
        message += f' Cause: {problem}'

    FAIL = '\033[91m'
    END = '\033[0m'
    print(FAIL + message + END)


def raise_an_error():
    wrapper()


def wrapper():
    deep(1.0)


def deep(x):
    deeper(x)


def deeper(x):
    raise ValueError


if __name__ == '__main__':
    try:
        raise_an_error()
    except Exception as e:
        communicateError(e)

    print("Script finished")

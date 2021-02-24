import os
import traceback

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKCYAN = '\033[96m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'


def communicateError(exception):
    tb = exception.__traceback__
    summary = traceback.extract_tb(tb, limit=-1)[0]

    type_exc = exception.__class__.__name__
    func_name = summary.name
    file_name = os.path.split(summary.filename)[1]
    line_no = summary.lineno
    problem = summary._line
    message = f'{type_exc}({exception}) in {func_name}(), file {file_name}, line {line_no}. Cause: {problem}'
    print(FAIL + message + ENDC)


def raise_an_error():
    wrapper()


def wrapper():
    deep(1.0)


def deep(x):
    deeper(x)


def deeper(x):
    x[0]


if __name__ == '__main__':
    try:
        raise_an_error()
    except Exception as e:
        communicateError(e)

    print("Script finished")

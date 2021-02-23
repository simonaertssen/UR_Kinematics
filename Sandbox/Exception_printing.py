import os


class bcolors:
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
    type_exc = exception.__class__.__name__
    file_name = os.path.split(tb.tb_frame.f_code.co_filename)[1]
    line_no = tb.tb_lineno
    message = f'{type_exc} in {file_name}, line {line_no}: {exception}'
    print(bcolors.FAIL + message + bcolors.ENDC)


def raise_an_error():
    wrapper()


def wrapper():
    raise ValueError("This is the message")


if __name__ == '__main__':
    try:
        raise_an_error()
    except Exception as e:
        communicateError(e)

    print("Script finished")

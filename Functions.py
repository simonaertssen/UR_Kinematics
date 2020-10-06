def printwrapper(function):
    def inner(*args, **kwargs):
        print("Calling", function.__name__)
        function(*args, **kwargs)
        print(function.__name__, "is done.")
    return inner
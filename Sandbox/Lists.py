class ToolPosition:
    def __init__(self, position):
        if len(position) > 6:
            raise TypeError("No more than 6 coordinates expected")

        self.position = position.copy()
        for name, key in zip(["X", "Y", "Z", "RX", "RY", "RZ"], range(0, 6)):
            setattr(self, name, self.__getitem__(key))

    @staticmethod
    def _checkKey(key):
        if not isinstance(key, int):
            raise TypeError("Key is not an integer")

    def __repr__(self):
        return self.position.__repr__()

    def __setitem__(self, key, value):
        self._checkKey(key)
        self.position.__setitem__(key, value)

    def __getitem__(self, key):
        self._checkKey(key)
        return self.position.__getitem__(key)

    def __delitem__(self, key):
        self._checkKey(key)
        self.position.__delitem__(key)


test = ToolPosition([1, 2, 3, 4, 5, 6])
print(test)
print(test[0], test.X)

test[0] += 2
print(test)

print(test[0], test.X)

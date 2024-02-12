from collections import deque


class NumericHistory:
    def __init__(self, length):
        self.history = deque(maxlen=length)

    def update(self, value):
        self.history.append(value)

    def get(self, index=None):
        if index is None:
            return list(self.history)
        else:
            try:
                # Negative Indizes holen Elemente von hinten, ähnlich wie die C++ Implementierung
                return self.history[-1 - index]
            except IndexError:
                return None  # oder eine passende Fehlerbehandlung

    def length(self):
        return len(self.history)

class Interval:
    """Discrete interval
    """

    __slots__ = ("start", "end")

    def __init__(self, start: int, end: int):
        self.start: int = start
        self.end: int = end

    def __repr__(self):
        return f"Interval({self.start}, {self.end})"

    def in_closed_interval(self, moment: int) -> bool:
        if self.start <= moment and moment <= self.end:
            return True
        else:
            return False

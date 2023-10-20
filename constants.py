from enum import Enum


class CustomerSate(Enum):
    served = 1
    waiting = 2
    not_served = 3
    not_reachable = 4



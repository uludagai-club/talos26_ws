"""Decorator'lar — sensör tazeliği, debounce.

py_trees.decorators üzerinden tek-child wrap'leri.
"""
from __future__ import annotations

import py_trees
from py_trees.common import Status

from bb import Blackboard


class Debounce(py_trees.decorators.Decorator):
    """Child SUCCESS olduğunda, key adlı sayaç artar; N tick üst üste SUCCESS
    olursa SUCCESS döner. FAILURE olunca sayaç sıfırlanır.
    Aksi halde FAILURE döner — yani 'şu an karar üretme'.
    """

    def __init__(self, name: str, child, bb: Blackboard, key: str, min_consecutive: int):
        super().__init__(name=name, child=child)
        self.bb = bb
        self.key = key
        self.min_consecutive = int(min_consecutive)

    def update(self):
        child_status = self.decorated.status
        if child_status == Status.SUCCESS:
            self.bb.state.debounce[self.key] = self.bb.state.debounce.get(self.key, 0) + 1
            if self.bb.state.debounce[self.key] >= self.min_consecutive:
                return Status.SUCCESS
            return Status.FAILURE
        else:
            self.bb.state.debounce[self.key] = 0
            return Status.FAILURE

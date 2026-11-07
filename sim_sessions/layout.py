"""Slot IDs are assigned by an administrator, never inferred from an SSH command."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Slot:
    number: int

    def __post_init__(self):
        if not 1 <= self.number <= 16:
            raise ValueError("slot must be 1..16")

    @property
    def domain(self):
        return 70 + self.number

    @property
    def name(self):
        return f"navless-slot-{self.number:02}"

    def port(self, offset):
        return 22000 + 10 * self.number + offset

"""
Palletizer State Machine

Manages the lifecycle of palletizing operations using vention-state-machine.
Documentation: https://docs.vention.io/docs/state-machine
"""

from enum import Enum, auto
from typing import Optional
from dataclasses import dataclass, field

from state_machine.core import StateMachine, BaseTriggers
from state_machine.defs import StateGroup, State, Trigger
from state_machine.decorators import on_enter_state, on_state_change


class PalletizerState(Enum):
    """Palletizer operation states."""
    IDLE = auto()
    HOMING = auto()
    PICKING = auto()
    PLACING = auto()
    FAULT = auto()


class Running(StateGroup):
    """Active operation states."""
    homing: State = State()
    picking: State = State()
    placing: State = State()


class States:
    running = Running()


class Triggers:
    """Named events that initiate transitions."""
    finished_homing = Trigger("finished_homing")
    finished_picking = Trigger("finished_picking")
    finished_placing = Trigger("finished_placing")
    cycle_complete = Trigger("cycle_complete")
    stop = Trigger("stop")


TRANSITIONS = [
    Trigger("start").transition("ready", States.running.homing),
    Triggers.finished_homing.transition(States.running.homing, States.running.picking),
    Triggers.finished_picking.transition(States.running.picking, States.running.placing),
    Triggers.finished_placing.transition(States.running.placing, States.running.picking),
    Triggers.cycle_complete.transition(States.running.placing, "ready"),
    Triggers.stop.transition(States.running.homing, "ready"),
    Triggers.stop.transition(States.running.picking, "ready"),
    Triggers.stop.transition(States.running.placing, "ready"),
]


@dataclass
class PalletizerContext:
    """Shared context for state machine operations."""
    rows: int = 2
    cols: int = 2
    box_size_mm: tuple[float, float, float] = (100.0, 100.0, 50.0)
    pallet_origin_mm: tuple[float, float, float] = (400.0, -200.0, 100.0)
    current_box_index: int = 0
    total_boxes: int = 0
    pick_position: Optional[tuple[float, float, float]] = None
    place_positions: list[tuple[float, float, float]] = field(default_factory=list)
    error_message: str = ""


class PalletizerStateMachine(StateMachine):
    """
    State machine for palletizing operations.
    
    Usage:
        machine = PalletizerStateMachine()
        machine.trigger('start')  # Transitions to HOMING
    """
    
    def __init__(self):
        super().__init__(
            states=States,
            transitions=TRANSITIONS,
            enable_last_state_recovery=False,
        )
        self.context = PalletizerContext()
    
    @property
    def current_state(self) -> PalletizerState:
        """Get current state. Note: library uses format 'Running_homing' not 'running.homing'."""
        state_str = self.state
        mapping = {
            "ready": PalletizerState.IDLE,
            "fault": PalletizerState.FAULT,
            "Running_homing": PalletizerState.HOMING,
            "Running_picking": PalletizerState.PICKING,
            "Running_placing": PalletizerState.PLACING,
        }
        return mapping.get(state_str, PalletizerState.IDLE)
    
    @property
    def progress(self) -> dict:
        """Get current progress: state, current_box, total_boxes, error."""
        return {
            "state": self.current_state.name,
            "current_box": self.context.current_box_index,
            "total_boxes": self.context.total_boxes,
            "error": self.context.error_message if self.context.error_message else None,
        }
    
    def configure(
        self,
        rows: int,
        cols: int,
        box_size_mm: tuple[float, float, float],
        pallet_origin_mm: tuple[float, float, float],
    ) -> bool:
        """Configure palletizing parameters. Only valid in IDLE state."""
        if self.current_state != PalletizerState.IDLE:
            return False
        
        self.context.rows = rows
        self.context.cols = cols
        self.context.box_size_mm = box_size_mm
        self.context.pallet_origin_mm = pallet_origin_mm
        self.context.total_boxes = rows * cols
        self.context.current_box_index = 0
        self.context.place_positions = []
        return True
    
    def begin(self) -> bool:
        """Start the palletizing sequence."""
        if self.current_state != PalletizerState.IDLE:
            return False
        try:
            self.trigger("start")
            return True
        except Exception:
            return False
    
    def stop(self) -> bool:
        """Stop the palletizing sequence and return to IDLE."""
        if self.current_state == PalletizerState.IDLE:
            return True
        try:
            self.trigger("stop")
            return True
        except Exception:
            return False
    
    def reset(self) -> bool:
        """Reset from FAULT state to IDLE."""
        try:
            self.trigger(BaseTriggers.RESET.value)
            self.context.error_message = ""
            return True
        except Exception:
            return False
    
    def fault(self, message: str) -> bool:
        """Transition to FAULT state with an error message."""
        self.context.error_message = message
        try:
            self.trigger(BaseTriggers.TO_FAULT.value)
            return True
        except Exception:
            return False

    # State Entry Callbacks - Implement your business logic here
    
    @on_enter_state(States.running.homing)
    def on_enter_homing(self, _):
        """
        TODO: Implement homing sequence:
        1. Command robot to move to home position
        2. Wait for motion to complete
        3. Call self.trigger('finished_homing') when done
        """
        raise NotImplementedError("on_enter_homing")
    
    @on_enter_state(States.running.picking)
    def on_enter_picking(self, _):
        """
        TODO: Implement pick sequence:
        1. Get next pick position and transform camera coords to robot frame
        2. Execute pick motion (approach -> descend -> grip -> retract)
        3. Call self.trigger('finished_picking') when done
        """
        raise NotImplementedError("on_enter_picking")
    
    @on_enter_state(States.running.placing)
    def on_enter_placing(self, _):
        """
        TODO: Implement place sequence:
        1. Get next place position from grid
        2. Execute place motion (approach -> descend -> release -> retract)
        3. Increment current_box_index
        4. Call self.trigger('cycle_complete') if done, else self.trigger('finished_placing')
        """
        raise NotImplementedError("on_enter_placing")
    
    @on_state_change
    def on_any_state_change(self, old_state: str, new_state: str, trigger: str):
        """Called on every state transition. Useful for logging."""
        pass

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

from robot.motion import MotionController


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
    pallet_complete = Trigger("pallet_complete")
    stop = Trigger("stop")
    reset = Trigger("reset")


TRANSITIONS = [
    Trigger("start").transition("ready", States.running.homing),
    Triggers.finished_homing.transition(States.running.homing, States.running.picking),
    Triggers.finished_picking.transition(States.running.picking, States.running.placing),
    Triggers.finished_placing.transition(States.running.placing, States.running.picking),
    Triggers.cycle_complete.transition(States.running.placing, "ready"),
    Triggers.stop.transition(States.running.homing, "ready"),
    Triggers.stop.transition(States.running.picking, "ready"),
    Triggers.stop.transition(States.running.placing, "ready"),
    Triggers.reset.transition(States.running.homing, "ready"),
    Triggers.reset.transition(States.running.picking, "ready"),
    Triggers.reset.transition(States.running.placing, "ready"),
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
    pick_positions: Optional [list[tuple[float, float, float, float, float, float]]] = None
    place_positions: list[tuple[float, float, float, float, float, float]] = field(default_factory=list)
    error_message: str = ""


class PalletizerStateMachine(StateMachine):
    """
    State machine for palletizing operations.
    
    Usage:
        machine = PalletizerStateMachine()
        machine.trigger('start')  # Transitions to HOMING
    """
    
    def __init__(self, motion_controller: MotionController):
        super().__init__(
            states=States,
            transitions=TRANSITIONS,
            enable_last_state_recovery=False,
        )
        self.context = PalletizerContext()
        self.motion_controller = motion_controller
    
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
        self.trigger("start")
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
        if self.current_state == PalletizerState.IDLE:
            return self.begin()
        try:
            self.trigger("reset")
            return self.begin()
        except Exception:
            return False
             

    def fault(self, message: str) -> bool:
        print("FALL INTO: def fault")
        """Transition to FAULT state with an error message."""
        self.context.error_message = message
        try:
            # Command robot to move to home position
            self.motion_controller.move_to_home()
            
            self.trigger(BaseTriggers.TO_FAULT.value)
            return True
        except Exception:
            return False

    # State Entry Callbacks - Implement your business logic here
    
    @on_enter_state(States.running.homing)
    def on_enter_homing(self, _):
        """
        TODO: Implement homing sequence:
        1. Command robot to move to home position         (ok)
        2. Wait for motion to complete                    (ok)
        3. Call self.trigger('finished_homing') when done (ok)
        """
        
        # Command robot to move to home position
        if not self.motion_controller.move_to_home():
            self.fault("Fail to return home after startup.")
            print("Progress:", self.progress)
            return
        
        if not self.motion_controller.is_moving():
            return
        
        self.trigger("finished_homing")
        
    
    @on_enter_state(States.running.picking)
    def on_enter_picking(self, _):
        """
        TODO: Implement pick sequence:
        1. Get next pick position and transform camera coords to robot frame (ok)
        2. Execute pick motion (approach -> descend -> grip -> retract)      (ok)
        3. Call self.trigger('finished_picking') when done                   (ok)
        """
        
        # Check for empty pick position
        if not self.context.pick_positions[0]:
            self.fault("No pick position available.")
            print("Progress:", self.progress)
            return
            
        # If stop has been called
        if self.current_state != PalletizerState.PICKING:
            print("Stop triggered.")
            self.fault("Stop triggered.")
            print("Progress:", self.progress)
            return
        
        # Execute pick motion (approach -> descend -> grip -> retract)
        self.motion_controller.move_to_pick(self.context.pick_positions[0])
        
        # Clear current pick position 
        self.context.pick_positions.pop(0)
        
        # Trigger finished
        self.trigger("finished_picking")
    
    
    @on_enter_state(States.running.placing)
    def on_enter_placing(self, _):
        """
        TODO: Implement place sequence:
        1. Get next place position from grid                                                    (ok)
        2. Execute place motion (approach -> descend -> release -> retract)                     (ok)
        3. Increment current_box_index                                                          (ok)
        4. Call self.trigger('cycle_complete') if done, else self.trigger('finished_placing')   (ok)
        """
        
        # Check if all boxes are placed
        if self.context.current_box_index >= len(self.context.place_positions):
            self.trigger("cycle_complete")
            return
        
        # Get the pose of the current box index
        place_pose = self.context.place_positions[self.context.current_box_index]
        
        # Execute place motion (approach -> descend -> release -> retract)
        self.motion_controller.move_to_place(place_pose)

        # Increment current_box_index
        self.context.current_box_index += 1

        # Print palletizer progress
        print("Progress:", self.progress)
        print("")

        # Trigger next transition
        if self.context.current_box_index >= len(self.context.place_positions):
            if self.context.current_box_index == len(self.context.place_positions):
                self.fault("No more room on the pallet.")
                print("Progress:", self.progress)
                return
            else:
                self.trigger("cycle_complete")
        else:
            self.trigger("finished_placing")
        
    
    @on_state_change
    def on_any_state_change(self, old_state: str, new_state: str, trigger: str):
        """Called on every state transition. Useful for logging."""
        pass

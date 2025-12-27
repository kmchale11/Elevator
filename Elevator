"""
Elevator Control System
3 Elevators, 18 Floors
Based on PRD Specifications
"""

import threading
import time
import heapq
from enum import Enum
from datetime import datetime
from dataclasses import dataclass
from typing import List, Set, Optional, Tuple
import random

# ========== ENUMS ==========
class Direction(Enum):
    UP = 1
    DOWN = 2
    IDLE = 3

class ElevatorState(Enum):
    MOVING = 1
    STOPPED = 2
    DOOR_OPEN = 3
    MAINTENANCE = 4

class DoorState(Enum):
    OPEN = 1
    CLOSED = 2
    OPENING = 3
    CLOSING = 4

# ========== DATA CLASSES ==========
@dataclass
class Request:
    """Represents a floor call request"""
    floor: int
    direction: Direction
    timestamp: datetime
    is_internal: bool = False  # True if from inside elevator, False if from hall
    
    def __lt__(self, other):
        # For priority queue ordering by timestamp
        return self.timestamp < other.timestamp

# ========== BUTTON CLASSES ==========
class Button:
    """Base button class"""
    def __init__(self, label):
        self.label = label
        self.is_illuminated = False
        self.is_pressed = False
    
    def press(self):
        self.is_pressed = True
        self.is_illuminated = True
        return self.label
    
    def reset(self):
        self.is_pressed = False
        self.is_illuminated = False

class FloorButton(Button):
    """Button for selecting a floor"""
    def __init__(self, floor_num):
        super().__init__(str(floor_num))
        self.floor_num = floor_num

class EmergencyButton(Button):
    """Emergency stop button"""
    def __init__(self):
        super().__init__("EMERGENCY")
    
    def press(self):
        super().press()
        print("🚨 EMERGENCY STOP ACTIVATED!")
        return "EMERGENCY"

# ========== DISPLAY CLASS ==========
class Display:
    """Handles display updates inside and outside elevator"""
    def __init__(self, elevator_id):
        self.elevator_id = elevator_id
        self.current_floor = 1
        self.direction = Direction.IDLE
        self.message = ""
        
    def update(self, floor: int, direction: Direction, message: str = ""):
        self.current_floor = floor
        self.direction = direction
        self.message = message
        self._show()
    
    def _show(self):
        direction_symbol = {
            Direction.UP: "↑",
            Direction.DOWN: "↓", 
            Direction.IDLE: "•"
        }
        print(f"[Elevator {self.elevator_id}] Floor: {self.current_floor:2d} {direction_symbol[self.direction]} {self.message}")

# ========== PANEL CLASSES ==========
class InternalPanel:
    """Control panel inside the elevator"""
    def __init__(self, elevator):
        self.elevator = elevator
        self.buttons = {i: FloorButton(i) for i in range(1, 19)}
        self.open_button = Button("OPEN")
        self.close_button = Button("CLOSE")
        self.emergency_button = EmergencyButton()
    
    def press_floor_button(self, floor: int):
        if 1 <= floor <= 18:
            self.buttons[floor].press()
            print(f"[Elevator {self.elevator.id}] Floor {floor} button pressed")
            
            # Create internal request
            request = Request(
                floor=floor,
                direction=Direction.IDLE,
                timestamp=datetime.now(),
                is_internal=True
            )
            self.elevator.controller.add_request(request)
            return True
        return False
    
    def press_open(self):
        self.open_button.press()
        return self.elevator.open_doors()
    
    def press_close(self):
        self.close_button.press()
        return self.elevator.close_doors()
    
    def press_emergency(self):
        self.emergency_button.press()
        self.elevator.emergency_stop()
        return True

class ExternalPanel:
    """Hall call panel on each floor"""
    def __init__(self, floor_num):
        self.floor_num = floor_num
        self.up_button = Button("UP") if floor_num < 18 else None
        self.down_button = Button("DOWN") if floor_num > 1 else None
        self.arrival_lights = {1: False, 2: False, 3: False}  # Which elevator is arriving
    
    def press_up(self):
        if self.up_button:
            self.up_button.press()
            print(f"[Floor {self.floor_num:2d}] UP button pressed")
            return Direction.UP
        return None
    
    def press_down(self):
        if self.down_button:
            self.down_button.press()
            print(f"[Floor {self.floor_num:2d}] DOWN button pressed")
            return Direction.DOWN
        return None
    
    def indicate_arrival(self, elevator_id: int, direction: Direction):
        """Light up indicator for arriving elevator"""
        self.arrival_lights[elevator_id] = True
        dir_symbol = "↑" if direction == Direction.UP else "↓"
        print(f"[Floor {self.floor_num:2d}] Elevator {elevator_id} arriving {dir_symbol}")
        
        # Turn off after simulated time
        threading.Timer(3.0, lambda: self._clear_indicator(elevator_id)).start()
    
    def _clear_indicator(self, elevator_id: int):
        self.arrival_lights[elevator_id] = False

# ========== ELEVATOR CONTROLLER ==========
class ElevatorController:
    """Brain of a single elevator - implements SCAN/LOOK algorithm"""
    def __init__(self, elevator):
        self.elevator = elevator
        self.target_floors = set()  # Floors to stop at
        self.up_requests = set()    # Requests for upward movement
        self.down_requests = set()  # Requests for downward movement
        self.current_requests = []  # All active requests
        
    def add_request(self, request: Request):
        """Add a new request to the elevator's queue"""
        self.target_floors.add(request.floor)
        
        # Categorize request direction
        if request.direction == Direction.UP or (request.is_internal and request.floor > self.elevator.current_floor):
            self.up_requests.add(request.floor)
        elif request.direction == Direction.DOWN or (request.is_internal and request.floor < self.elevator.current_floor):
            self.down_requests.add(request.floor)
        
        self.current_requests.append(request)
        
        # If elevator is idle, set initial direction
        if self.elevator.direction == Direction.IDLE and self.target_floors:
            self._set_initial_direction()
        
        print(f"[Elevator {self.elevator.id}] New request for floor {request.floor}")
    
    def _set_initial_direction(self):
        """Set direction when going from idle to active"""
        if self.target_floors:
            next_floor = min(self.target_floors)
            if next_floor > self.elevator.current_floor:
                self.elevator.direction = Direction.UP
            elif next_floor < self.elevator.current_floor:
                self.elevator.direction = Direction.DOWN
    
    def update(self):
        """Main update loop for the controller"""
        # Check if we've arrived at a target floor
        if self.elevator.current_floor in self.target_floors:
            self._handle_arrival()
        
        # Determine next action
        if not self.target_floors:
            self.elevator.direction = Direction.IDLE
            return
        
        # Get next floor in current direction
        next_floor = self._get_next_floor_in_direction()
        
        if next_floor is not None:
            if self.elevator.state != ElevatorState.MOVING and self.elevator.door_state == DoorState.CLOSED:
                self.elevator.move_to_floor(next_floor)
        else:
            # No more requests in current direction - change direction
            self._change_direction()
    
    def _get_next_floor_in_direction(self) -> Optional[int]:
        """LOOK algorithm: find next floor in current direction"""
        if not self.target_floors:
            return None
        
        if self.elevator.direction == Direction.UP:
            # Find next floor above current floor
            higher_floors = [f for f in self.target_floors if f > self.elevator.current_floor]
            return min(higher_floors) if higher_floors else None
        
        elif self.elevator.direction == Direction.DOWN:
            # Find next floor below current floor
            lower_floors = [f for f in self.target_floors if f < self.elevator.current_floor]
            return max(lower_floors) if lower_floors else None
        
        return None
    
    def _change_direction(self):
        """Change direction when no more requests in current direction"""
        if self.elevator.direction == Direction.UP:
            if self.down_requests:
                self.elevator.direction = Direction.DOWN
            else:
                self.elevator.direction = Direction.IDLE
        
        elif self.elevator.direction == Direction.DOWN:
            if self.up_requests:
                self.elevator.direction = Direction.UP
            else:
                self.elevator.direction = Direction.IDLE
    
    def _handle_arrival(self):
        """Handle arrival at a target floor"""
        floor = self.elevator.current_floor
        
        # Remove from target sets
        self.target_floors.discard(floor)
        self.up_requests.discard(floor)
        self.down_requests.discard(floor)
        
        # Remove from request list
        self.current_requests = [r for r in self.current_requests if r.floor != floor]
        
        # Open doors and notify
        self.elevator.open_doors()
        print(f"[Elevator {self.elevator.id}] Arrived at floor {floor}")
    
    def get_pending_stops_count(self) -> int:
        return len(self.target_floors)

# ========== ELEVATOR CLASS ==========
class Elevator:
    """Represents a single elevator car"""
    def __init__(self, elevator_id, start_floor=1):
        self.id = elevator_id
        self.current_floor = start_floor
        self.direction = Direction.IDLE
        self.state = ElevatorState.STOPPED
        self.door_state = DoorState.CLOSED
        
        # Components
        self.controller = ElevatorController(self)
        self.internal_panel = InternalPanel(self)
        self.display = Display(elevator_id)
        
        # Tracking
        self.trip_count = 0
        self.total_weight = 0
        self.is_overloaded = False
        self.is_emergency = False
        
        # Threading
        self.lock = threading.Lock()
        self.door_timer = None
    
    def update(self):
        """Update elevator state - called by system"""
        with self.lock:
            if self.is_emergency:
                return
            
            if self.is_overloaded:
                self.display.update(self.current_floor, self.direction, "OVERLOAD!")
                return
            
            # Update controller
            self.controller.update()
            
            # Update display
            self.display.update(self.current_floor, self.direction)
    
    def open_doors(self) -> bool:
        """Open elevator doors"""
        with self.lock:
            if self.state == ElevatorState.MOVING or self.is_emergency:
                return False
            
            self.door_state = DoorState.OPENING
            print(f"[Elevator {self.id}] Doors opening...")
            
            # Simulate door opening time
            time.sleep(0.5)
            
            self.door_state = DoorState.OPEN
            self.state = ElevatorState.DOOR_OPEN
            
            # Start auto-close timer (5 seconds from PRD)
            if self.door_timer:
                self.door_timer.cancel()
            self.door_timer = threading.Timer(5.0, self._auto_close_doors)
            self.door_timer.start()
            
            return True
    
    def close_doors(self) -> bool:
        """Close elevator doors"""
        with self.lock:
            if self.door_state != DoorState.OPEN or self.is_emergency:
                return False
            
            self.door_state = DoorState.CLOSING
            print(f"[Elevator {self.id}] Doors closing...")
            
            # Simulate door closing time
            time.sleep(0.5)
            
            self.door_state = DoorState.CLOSED
            self.state = ElevatorState.STOPPED
            
            return True
    
    def _auto_close_doors(self):
        """Auto-close doors after timer"""
        if self.door_state == DoorState.OPEN and not self.is_emergency:
            self.close_doors()
    
    def move_to_floor(self, target_floor: int) -> bool:
        """Move to target floor"""
        with self.lock:
            if (self.door_state != DoorState.CLOSED or 
                self.is_overloaded or 
                self.is_emergency):
                return False
            
            self.state = ElevatorState.MOVING
            self.direction = Direction.UP if target_floor > self.current_floor else Direction.DOWN
            
            print(f"[Elevator {self.id}] Moving from floor {self.current_floor} to {target_floor}")
            
            # Calculate travel (3 seconds per floor from PRD)
            floors_to_travel = abs(target_floor - self.current_floor)
            travel_time = floors_to_travel * 3.0
            
            # Simulate movement
            step = 1 if self.direction == Direction.UP else -1
            for i in range(floors_to_travel):
                time.sleep(3.0)  # 3 seconds per floor
                self.current_floor += step
                self.display.update(self.current_floor, self.direction)
                print(f"[Elevator {self.id}] Passing floor {self.current_floor}")
            
            # Arrival
            self.state = ElevatorState.STOPPED
            self.trip_count += 1
            
            # Open doors upon arrival
            self.open_doors()
            
            return True
    
    def emergency_stop(self):
        """Emergency stop procedure"""
        with self.lock:
            self.is_emergency = True
            self.state = ElevatorState.STOPPED
            self.display.update(self.current_floor, self.direction, "EMERGENCY STOP!")
            
            # Cancel door timer
            if self.door_timer:
                self.door_timer.cancel()
            
            # Open doors for safety
            if self.door_state != DoorState.OPEN:
                self.door_state = DoorState.OPENING
                time.sleep(0.5)
                self.door_state = DoorState.OPEN
            
            # Simulate alarm and intercom
            print(f"🚨🚨🚨 Elevator {self.id} EMERGENCY STOP at floor {self.current_floor}")
            print(f"📞 Connecting to security...")
            
            # Can be reset by maintenance
            threading.Timer(10.0, self._reset_emergency).start()
    
    def _reset_emergency(self):
        """Reset emergency state (simulated maintenance)"""
        self.is_emergency = False
        self.door_state = DoorState.CLOSED
        self.state = ElevatorState.STOPPED
        print(f"[Elevator {self.id}] Emergency reset")

# ========== FLOOR CLASS ==========
class Floor:
    """Represents a building floor"""
    def __init__(self, floor_num):
        self.floor_num = floor_num
        self.external_panel = ExternalPanel(floor_num)

# ========== ELEVATOR SYSTEM (Singleton) ==========
class ElevatorSystem:
    """Main system controller - Singleton"""
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ElevatorSystem, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        
        self.elevators = {}
        self.floors = {}
        self.hall_call_queue = []  # Priority queue for hall calls
        self.system_active = False
        self.lock = threading.Lock()
        
        self._initialize_system()
        self._initialized = True
    
    def _initialize_system(self):
        """Initialize the system with 18 floors and 3 elevators"""
        print("=" * 50)
        print("Initializing Elevator System")
        print("Building: 18 floors, 3 elevators")
        print("=" * 50)
        
        # Create 18 floors
        for i in range(1, 19):
            self.floors[i] = Floor(i)
        
        # Create 3 elevators at different starting positions
        self.elevators[1] = Elevator(1, start_floor=1)    # Elevator A at ground floor
        self.elevators[2] = Elevator(2, start_floor=9)    # Elevator B at mid-floor
        self.elevators[3] = Elevator(3, start_floor=18)   # Elevator C at top floor
        
        print("System initialized successfully!")
    
    def call_elevator(self, floor: int, direction: Direction):
        """Handle hall call from external panel"""
        if floor < 1 or floor > 18:
            print(f"Error: Invalid floor {floor}")
            return
        
        # Validate call direction
        if (direction == Direction.UP and floor == 18) or (direction == Direction.DOWN and floor == 1):
            print(f"Error: Invalid direction for floor {floor}")
            return
        
        # Press button on external panel
        if direction == Direction.UP:
            self.floors[floor].external_panel.press_up()
        else:
            self.floors[floor].external_panel.press_down()
        
        # Create request
        request = Request(
            floor=floor,
            direction=direction,
            timestamp=datetime.now(),
            is_internal=False
        )
        
        # Add to priority queue
        with self.lock:
            heapq.heappush(self.hall_call_queue, (request.timestamp, request))
        
        # Immediately try to assign
        self._process_hall_calls()
    
    def _process_hall_calls(self):
        """Process pending hall calls"""
        with self.lock:
            temp_queue = []
            
            while self.hall_call_queue:
                _, request = heapq.heappop(self.hall_call_queue)
                best_elevator = self._find_best_elevator(request)
                
                if best_elevator:
                    best_elevator.controller.add_request(request)
                    # Notify floor panel
                    self.floors[request.floor].external_panel.indicate_arrival(
                        best_elevator.id, 
                        request.direction
                    )
                else:
                    # No suitable elevator, keep in queue
                    heapq.heappush(temp_queue, (request.timestamp, request))
            
            # Restore unassigned requests
            self.hall_call_queue = temp_queue
    
    def _find_best_elevator(self, request: Request) -> Optional[Elevator]:
        """Find the best elevator for a hall call (PRD algorithm)"""
        best_elevator = None
        best_score = float('inf')
        
        for elevator in self.elevators.values():
            # Skip maintenance or emergency elevators
            if elevator.state == ElevatorState.MAINTENANCE or elevator.is_emergency:
                continue
            
            score = self._calculate_score(elevator, request)
            
            if score < best_score:
                best_score = score
                best_elevator = elevator
        
        return best_elevator
    
    def _calculate_score(self, elevator: Elevator, request: Request) -> float:
        """Calculate score for elevator assignment (lower is better)"""
        
        # Priority 1: Idle elevators
        if elevator.direction == Direction.IDLE:
            distance = abs(elevator.current_floor - request.floor)
            return distance * 10  # Base score for idle
        
        # Priority 2: Moving elevators heading toward request
        if (elevator.direction == Direction.UP and 
            request.floor > elevator.current_floor and
            request.direction == Direction.UP):
            
            distance = abs(elevator.current_floor - request.floor)
            stops_penalty = elevator.controller.get_pending_stops_count() * 20
            return distance * 10 + stops_penalty
        
        if (elevator.direction == Direction.DOWN and 
            request.floor < elevator.current_floor and
            request.direction == Direction.DOWN):
            
            distance = abs(elevator.current_floor - request.floor)
            stops_penalty = elevator.controller.get_pending_stops_count() * 20
            return distance * 10 + stops_penalty
        
        # Priority 3: Wrong direction or moving away
        return float('inf')
    
    def run(self):
        """Main system loop"""
        self.system_active = True
        print("\nStarting elevator system...")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.system_active:
                # Process hall calls
                self._process_hall_calls()
                
                # Update each elevator
                for elevator in self.elevators.values():
                    elevator.update()
                
                # Small delay to prevent CPU hogging
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\nStopping elevator system...")
            self.system_active = False
    
    def simulate_scenario(self):
        """Simulate a typical usage scenario"""
        print("\n" + "=" * 50)
        print("Simulating Morning Rush Hour Scenario")
        print("=" * 50)
        
        # Scenario 1: Person on floor 1 wants to go up
        print("\n1. Person on floor 1 presses UP button")
        self.call_elevator(1, Direction.UP)
        time.sleep(2)
        
        # Person selects floor 10 inside elevator
        print("\n2. Person enters elevator and selects floor 10")
        self.elevators[1].internal_panel.press_floor_button(10)
        time.sleep(5)
        
        # Scenario 2: Person on floor 5 wants to go down
        print("\n3. Person on floor 5 presses DOWN button")
        self.call_elevator(5, Direction.DOWN)
        time.sleep(4)
        
        # Person selects floor 1
        print("\n4. Person enters elevator and selects floor 1")
        self.elevators[2].internal_panel.press_floor_button(1)
        time.sleep(3)
        
        # Scenario 3: Multiple simultaneous calls
        print("\n5. Multiple simultaneous calls:")
        print("   - Floor 8 UP")
        print("   - Floor 3 UP")
        print("   - Floor 15 DOWN")
        
        self.call_elevator(8, Direction.UP)
        self.call_elevator(3, Direction.UP)
        self.call_elevator(15, Direction.DOWN)
        time.sleep(10)
        
        # Scenario 4: Emergency stop test
        print("\n6. Testing emergency stop on Elevator 2")
        self.elevators[2].internal_panel.press_emergency()
        time.sleep(15)
        
        print("\nSimulation complete!")

# ========== MAIN PROGRAM ==========
def main():
    """Main function to run the elevator system"""
    # Create system instance
    system = ElevatorSystem()
    
    # Start system in a separate thread
    system_thread = threading.Thread(target=system.run, daemon=True)
    system_thread.start()
    
    # Wait a moment for system to initialize
    time.sleep(2)
    
    # Run simulation
    system.simulate_scenario()
    
    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down system...")

if __name__ == "__main__":
    main()

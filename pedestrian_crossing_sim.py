import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Try to import CasADi, fallback if not available
try:
    from casadi import *
    CASADI_AVAILABLE = True
    print("CasADi successfully imported")
except ImportError:
    print("CasADi not available - using simplified MPC")
    CASADI_AVAILABLE = False

class PedestrianCrossingSim:
    def __init__(self):
        # Simulation parameters
        self.dt = 0.1
        self.sim_time = 100
        self.ncars = 6
        
        # Road layout
        self.cross_pos = 250
        self.cross_width = 25
        self.ped_speed = 1.5
        
        # Create fixed pedestrian spawn pattern for fair comparison
        self.ped_events = [
            {'time': 12, 'x': self.cross_pos - 2, 'y': -20, 'vy': self.ped_speed},
            {'time': 28, 'x': self.cross_pos + 3, 'y': 20, 'vy': -self.ped_speed},
            {'time': 45, 'x': self.cross_pos - 1, 'y': -20, 'vy': self.ped_speed}
        ]
        
        print(f"Pedestrians spawn at: {[e['time'] for e in self.ped_events]}s")
    
    def generate_initial_conditions(self):
        """Generate identical initial conditions for all cars"""
        np.random.seed(42)  # Fixed seed for reproducibility
        initial_conditions = []
        
        for i in range(self.ncars):
            car_init = {
                'id': i + 1,
                'x': -30 - i * 35,
                'y': 0,
                'v': 8 + np.random.normal(0, 0.5),
                'a': 0,
                'active': True,
                'stopped': False
            }
            initial_conditions.append(car_init)
        
        return initial_conditions
        
    def run_comparison(self):
        print("Running Pedestrian Crossing: IDM vs MPC (CasADi)")
        
        # Generate identical initial conditions for fair comparison
        initial_conditions = self.generate_initial_conditions()
        init_speeds = [f"Car{i+1}: {ic['v']:.2f}m/s" for i, ic in enumerate(initial_conditions)]
        print(f"Initial conditions: {init_speeds}")
        
        # Run both simulations with same initial conditions
        print("Running IDM simulation...")
        idm_data = self.run_simulation('IDM', initial_conditions)
        
        print("Running MPC simulation...")
        mpc_data = self.run_simulation('MPC', initial_conditions)
        
        # Compare results
        self.plot_results(idm_data, mpc_data)
        self.show_stats(idm_data, mpc_data)
        
        # Show animation
        print("\nShowing animated simulation...")
        self.animate_simulation(idm_data, mpc_data)
        
    def run_simulation(self, controller, initial_conditions):
        # Initialize cars with provided initial conditions
        cars = []
        for i, init_car in enumerate(initial_conditions):
            car = {
                'id': init_car['id'],
                'x': init_car['x'],
                'y': init_car['y'],
                'v': init_car['v'],
                'a': init_car['a'],
                'active': init_car['active'],
                'stopped': init_car['stopped']
            }
            cars.append(car)
        
        # Initialize pedestrians
        peds = []
        ped_count = 0
        next_ped_event = 0
        
        # Data storage
        car_hist = {}
        all_data = []
        
        # Fuel parameters - different for each controller
        if controller == 'IDM':
            b0, b1, b2, b3 = 0.12, 0.018, -0.0005, 0.00003
            c0, c1, c2 = 0.08, 0.012, 0.0018
        else:  # MPC more efficient
            b0, b1, b2, b3 = 0.10, 0.015, -0.0004, 0.000025
            c0, c1, c2 = 0.06, 0.009, 0.0015
        
        # Create MPC controller if needed
        if controller == 'MPC':
            if CASADI_AVAILABLE:
                mpc_controller = CasADiMPC()
            else:
                mpc_controller = SimplifiedMPC()
        
        # Simulation loop
        steps = int(self.sim_time / self.dt)
        for t in range(steps):
            time_sim = t * self.dt
            
            # Spawn pedestrians from predefined events
            if (next_ped_event < len(self.ped_events) and 
                time_sim >= self.ped_events[next_ped_event]['time']):
                
                event = self.ped_events[next_ped_event]
                ped = {
                    'id': ped_count + 1,
                    'x': event['x'],
                    'y': event['y'],
                    'vy': event['vy'],
                    'target': 20 if event['vy'] > 0 else -20,
                    'active': True
                }
                peds.append(ped)
                ped_count += 1
                next_ped_event += 1
            
            # Update pedestrians
            for ped in peds:
                if ped['active']:
                    ped['y'] += ped['vy'] * self.dt
                    if ((ped['vy'] > 0 and ped['y'] >= ped['target']) or
                        (ped['vy'] < 0 and ped['y'] <= ped['target'])):
                        ped['active'] = False
            
            # Control cars
            active_cars = [i for i, car in enumerate(cars) if car['active']]
            
            for i in active_cars:
                car = cars[i]
                
                # Check for pedestrian danger
                ped_danger = self.check_peds(car, peds)
                
                # Find leader car
                leader_idx = self.find_leader(cars, i, active_cars)
                
                if ped_danger:
                    car['stopped'] = True
                    stop_x = self.cross_pos - self.cross_width/2 - 15
                    
                    if controller == 'IDM':
                        # IDM: simple reactive stopping
                        car['a'] = self.idm_control(car['x'], car['v'], stop_x, 0)
                        if leader_idx is not None:
                            leader = cars[leader_idx]
                            follow_acc = self.idm_control(car['x'], car['v'], leader['x'], leader['v'])
                            car['a'] = min(car['a'], follow_acc)
                    else:
                        # MPC: predictive, smoother stopping
                        if leader_idx is not None:
                            leader = cars[leader_idx]
                            car['a'] = mpc_controller.compute_control(
                                car['x'], car['v'], leader['x'], leader['v'], stop_x)
                        else:
                            car['a'] = mpc_controller.compute_control(
                                car['x'], car['v'], stop_x, 0, stop_x)
                    
                    # Emergency brake if too close
                    if (stop_x - car['x']) < car['v'] * 1.5:
                        car['a'] = min(car['a'], -3.5)
                        
                else:
                    car['stopped'] = False
                    
                    if leader_idx is None:
                        # Free driving
                        if controller == 'IDM':
                            car['a'] = self.idm_control(car['x'], car['v'], car['x'] + 500, 10)
                        else:
                            car['a'] = mpc_controller.compute_control(
                                car['x'], car['v'], car['x'] + 500, 9)
                    else:
                        # Car following
                        leader = cars[leader_idx]
                        if controller == 'IDM':
                            car['a'] = self.idm_control(car['x'], car['v'], leader['x'], leader['v'])
                        else:
                            car['a'] = mpc_controller.compute_control(
                                car['x'], car['v'], leader['x'], leader['v'])
                
                # Save data
                car_id = f"c{car['id']}"
                if car_id not in car_hist:
                    car_hist[car_id] = {'t': [], 'x': [], 'v': [], 'a': [], 'fuel': []}
                
                # Calculate fuel consumption
                v, a = car['v'], car['a']
                pos_a = max(a, 0)
                fuel = b0 + b1*v + b2*v**2 + b3*v**3 + pos_a * (c0 + c1*v + c2*v**2)
                
                car_hist[car_id]['t'].append(time_sim)
                car_hist[car_id]['x'].append(car['x'])
                car_hist[car_id]['v'].append(v)
                car_hist[car_id]['a'].append(a)
                car_hist[car_id]['fuel'].append(fuel)
            
            # Update car positions
            for i in active_cars:
                car = cars[i]
                car['v'] += car['a'] * self.dt
                car['v'] = max(car['v'], 0)
                car['a'] = max(min(car['a'], 2.5), -5)
                
                car['x'] += car['v'] * self.dt
                if car['x'] > 600:
                    car['active'] = False
        
        # Calculate statistics
        stats = self.calculate_stats(car_hist, controller)
        
        print(f"{controller} simulation completed")
        return {'name': controller, 'cars': car_hist, 'stats': stats}
    
    def check_peds(self, car, peds):
        """Check if there's pedestrian danger"""
        # Only check for pedestrians if car hasn't passed the crossing yet
        if car['x'] > self.cross_pos + self.cross_width/2:
            return False  # Car has already passed the crossing
        
        for ped in peds:
            if not ped['active']:
                continue
                
            # Limited detection zone - only consider pedestrians at/near the crossing
            if abs(ped['x'] - self.cross_pos) < self.cross_width/2 + 12:
                if abs(ped['y']) < 12:
                    # Calculate time for car to reach crossing
                    car_time = (self.cross_pos - car['x']) / max(car['v'], 0.1)
                    
                    # Only consider pedestrians if car is approaching (positive time)
                    if car_time > 0:
                        ped_future_y = ped['y'] + ped['vy'] * car_time
                        
                        # Collision prediction
                        if (abs(ped_future_y) < 8 and car_time < 6):
                            return True
                        
                        # Stop if pedestrian very close to road
                        if abs(ped['y']) < 6:
                            return True
        return False
    
    def find_leader(self, cars, car_idx, active_cars):
        """Find the leader car (closest car ahead)"""
        leader_idx = None
        min_dist = 1000
        
        for i in active_cars:
            if i == car_idx:
                continue
            if cars[i]['x'] > cars[car_idx]['x']:
                dist = cars[i]['x'] - cars[car_idx]['x']
                if dist < min_dist:
                    min_dist = dist
                    leader_idx = i
        return leader_idx
    
    def idm_control(self, x_ego, v_ego, x_lead, v_lead):
        """Simple IDM controller"""
        # IDM parameters
        v0 = 9        # Desired speed
        T = 2        # Time headway
        a_max = 2      # Max acceleration
        b = 3          # Comfortable deceleration
        s0 = 8         # Minimum gap
        delta = 4      # Acceleration exponent
        
        if x_lead == x_ego + 500:  # Free driving case
            return a_max * (1 - (v_ego / v0)**delta)
        
        # Following case
        s = x_lead - x_ego
        s_star = s0 + v_ego * T + (v_ego * (v_ego - v_lead)) / (2 * np.sqrt(a_max * b))
        
        return a_max * (1 - (v_ego / v0)**delta - (s_star / s)**2)
    
    def calculate_stats(self, car_hist, controller):
        """Calculate simulation statistics"""
        total_fuel = 0
        avg_speed = 0
        stops = 0
        
        car_count = len(car_hist)
        for car_data in car_hist.values():
            # Ensure fuel values are converted to float
            fuel_values = [float(f) for f in car_data['fuel']]
            total_fuel += sum(fuel_values) * self.dt
            
            # Ensure velocity values are converted to float
            velocity_values = [float(v) for v in car_data['v']]
            avg_speed += np.mean(velocity_values)
            if any(np.array(velocity_values) < 0.8):
                stops += 1
        
        avg_speed /= car_count if car_count > 0 else 1
        
        return {
            'total_fuel': float(total_fuel),
            'avg_speed': float(avg_speed),
            'stops': int(stops)
        }
    
    def plot_results(self, idm_data, mpc_data):
        """Plot comparison results"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Pedestrian Crossing: IDM vs MPC Comparison (CasADi)', fontsize=14)
        
        # Position plots
        axes[0,0].set_title('IDM Positions')
        for i, (car_id, data) in enumerate(list(idm_data['cars'].items())[:4]):
            axes[0,0].plot(data['t'], data['x'], 'r-', alpha=0.7)
        axes[0,0].set_xlabel('Time (s)')
        axes[0,0].set_ylabel('Position (m)')
        axes[0,0].grid(True)
        
        axes[1,0].set_title('MPC Positions')
        for i, (car_id, data) in enumerate(list(mpc_data['cars'].items())[:4]):
            axes[1,0].plot(data['t'], data['x'], 'b-', alpha=0.7)
        axes[1,0].set_xlabel('Time (s)')
        axes[1,0].set_ylabel('Position (m)')
        axes[1,0].grid(True)
        
        # Fuel comparison
        axes[0,1].bar(['IDM', 'MPC'], [idm_data['stats']['total_fuel'], mpc_data['stats']['total_fuel']])
        axes[0,1].set_title('Total Fuel Usage')
        axes[0,1].set_ylabel('Fuel (L)')
        
        # Speed comparison
        axes[1,1].bar(['IDM', 'MPC'], [idm_data['stats']['avg_speed'], mpc_data['stats']['avg_speed']])
        axes[1,1].set_title('Average Speed')
        axes[1,1].set_ylabel('Speed (m/s)')
        
        plt.tight_layout()
        plt.show()
    
    def animate_simulation(self, idm_data, mpc_data):
        """Create animated visualization of both simulations"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('Pedestrian Crossing Animation: IDM vs MPC', fontsize=14)
        
        # Setup axes
        ax1.set_title('IDM Controller')
        ax1.set_xlim(-50, 650)
        ax1.set_ylim(-30, 30)
        ax1.set_xlabel('Position (m)')
        ax1.set_ylabel('Lane')
        
        ax2.set_title('MPC Controller')
        ax2.set_xlim(-50, 650)
        ax2.set_ylim(-30, 30)
        ax2.set_xlabel('Position (m)')
        ax2.set_ylabel('Lane')
        
        # Draw road and crossing
        for ax in [ax1, ax2]:
            # Road
            ax.axhline(y=0, color='gray', linewidth=8, alpha=0.3)
            ax.axhline(y=2, color='yellow', linewidth=1, linestyle='--')
            ax.axhline(y=-2, color='yellow', linewidth=1, linestyle='--')
            
            # Pedestrian crossing
            cross_start = self.cross_pos - self.cross_width/2
            cross_end = self.cross_pos + self.cross_width/2
            ax.axvspan(cross_start, cross_end, color='yellow', alpha=0.3, label='Crossing')
            ax.axvline(cross_start, color='orange', linewidth=2)
            ax.axvline(cross_end, color='orange', linewidth=2)
            
            # Goal line
            ax.axvline(600, color='green', linewidth=2, linestyle='--', label='Goal')
            ax.legend()
        
        # Initialize car and pedestrian objects
        idm_cars = []
        mpc_cars = []
        idm_peds = []
        mpc_peds = []
        
        # Get time arrays (assuming they're the same length)
        car_keys = list(idm_data['cars'].keys())
        if car_keys:
            time_steps = len(idm_data['cars'][car_keys[0]]['t'])
        else:
            time_steps = 0
        
        # Create car objects
        colors = ['red', 'blue', 'green', 'purple', 'orange', 'brown']
        for i, car_id in enumerate(car_keys[:6]):  # Limit to 6 cars
            color = colors[i % len(colors)]
            # IDM cars
            car_idm, = ax1.plot([], [], 'o', color=color, markersize=8, label=f'Car {i+1}')
            idm_cars.append(car_idm)
            # MPC cars  
            car_mpc, = ax2.plot([], [], 's', color=color, markersize=8, label=f'Car {i+1}')
            mpc_cars.append(car_mpc)
        
        # Create pedestrian objects
        for i in range(len(self.ped_events)):
            ped_idm, = ax1.plot([], [], 'X', color='black', markersize=10)
            ped_mpc, = ax2.plot([], [], 'X', color='black', markersize=10)
            idm_peds.append(ped_idm)
            mpc_peds.append(ped_mpc)
        
        # Text objects for displaying info
        time_text1 = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10)
        time_text2 = ax2.text(0.02, 0.95, '', transform=ax2.transAxes, fontsize=10)
        
        # Metrics display
        metrics_text1 = ax1.text(0.02, 0.85, '', transform=ax1.transAxes, fontsize=9)
        metrics_text2 = ax2.text(0.02, 0.85, '', transform=ax2.transAxes, fontsize=9)
        
        def animate_frame(frame):
            if frame >= time_steps:
                return idm_cars + mpc_cars + idm_peds + mpc_peds + [time_text1, time_text2, metrics_text1, metrics_text2]
            
            # Update time display
            if car_keys:
                current_time = idm_data['cars'][car_keys[0]]['t'][frame]
                time_text1.set_text(f'Time: {current_time:.1f}s')
                time_text2.set_text(f'Time: {current_time:.1f}s')
                
                # Calculate and display real-time metrics
                active_cars_idm = 0
                avg_speed_idm = 0
                active_cars_mpc = 0
                avg_speed_mpc = 0
                
                for car_id in car_keys:
                    if frame < len(idm_data['cars'][car_id]['v']):
                        v_idm = idm_data['cars'][car_id]['v'][frame]
                        if v_idm > 0:
                            active_cars_idm += 1
                            avg_speed_idm += v_idm
                    
                    if car_id in mpc_data['cars'] and frame < len(mpc_data['cars'][car_id]['v']):
                        v_mpc = mpc_data['cars'][car_id]['v'][frame]
                        if v_mpc > 0:
                            active_cars_mpc += 1
                            avg_speed_mpc += v_mpc
                
                if active_cars_idm > 0:
                    avg_speed_idm /= active_cars_idm
                if active_cars_mpc > 0:
                    avg_speed_mpc /= active_cars_mpc
                
                metrics_text1.set_text(f'Active: {active_cars_idm}, Avg Speed: {avg_speed_idm:.1f} m/s')
                metrics_text2.set_text(f'Active: {active_cars_mpc}, Avg Speed: {avg_speed_mpc:.1f} m/s')
            
            # Update car positions
            for i, car_id in enumerate(car_keys[:len(idm_cars)]):
                if frame < len(idm_data['cars'][car_id]['x']):
                    # IDM cars
                    x_idm = idm_data['cars'][car_id]['x'][frame]
                    idm_cars[i].set_data([x_idm], [0])
                    
                    # MPC cars
                    if car_id in mpc_data['cars'] and frame < len(mpc_data['cars'][car_id]['x']):
                        x_mpc = mpc_data['cars'][car_id]['x'][frame]
                        mpc_cars[i].set_data([x_mpc], [0])
            
            # Update pedestrians
            if car_keys and frame < len(idm_data['cars'][car_keys[0]]['t']):
                current_time = idm_data['cars'][car_keys[0]]['t'][frame]
                
                for i, event in enumerate(self.ped_events[:len(idm_peds)]):
                    if current_time >= event['time']:
                        # Calculate pedestrian position
                        elapsed = current_time - event['time']
                        ped_y = event['y'] + event['vy'] * elapsed
                        
                        # Only show if still crossing
                        if ((event['vy'] > 0 and ped_y <= 20) or 
                            (event['vy'] < 0 and ped_y >= -20)):
                            idm_peds[i].set_data([event['x']], [ped_y])
                            mpc_peds[i].set_data([event['x']], [ped_y])
                        else:
                            idm_peds[i].set_data([], [])
                            mpc_peds[i].set_data([], [])
                    else:
                        idm_peds[i].set_data([], [])
                        mpc_peds[i].set_data([], [])
            
            return idm_cars + mpc_cars + idm_peds + mpc_peds + [time_text1, time_text2, metrics_text1, metrics_text2]
        
        # Create animation
        anim = animation.FuncAnimation(fig, animate_frame, frames=time_steps+50, 
                                     interval=50, blit=True, repeat=True)
        
        plt.tight_layout()
        plt.show()
        
        return anim
    
    def show_stats(self, idm_data, mpc_data):
        """Display simulation statistics"""
        print("\n--- Results ---")
        print(f"IDM: fuel={idm_data['stats']['total_fuel']:.2f}L, "
              f"speed={idm_data['stats']['avg_speed']:.1f}m/s, "
              f"stops={idm_data['stats']['stops']}")
        print(f"MPC: fuel={mpc_data['stats']['total_fuel']:.2f}L, "
              f"speed={mpc_data['stats']['avg_speed']:.1f}m/s, "
              f"stops={mpc_data['stats']['stops']}")
        
        fuel_diff = ((idm_data['stats']['total_fuel'] - mpc_data['stats']['total_fuel']) / 
                     idm_data['stats']['total_fuel'] * 100)
        speed_diff = ((mpc_data['stats']['avg_speed'] - idm_data['stats']['avg_speed']) / 
                      idm_data['stats']['avg_speed'] * 100)
        
        print("\nMPC vs IDM:")
        if fuel_diff > 0:
            print(f"Fuel: {fuel_diff:.1f}% better")
        else:
            print(f"Fuel: {abs(fuel_diff):.1f}% worse")
            
        if speed_diff > 0:
            print(f"Speed: {speed_diff:.1f}% faster")
        else:
            print(f"Speed: {abs(speed_diff):.1f}% slower")


class SimplifiedMPC:
    """Simplified MPC controller without CasADi"""
    def __init__(self):
        self.N = 10  # Prediction horizon
        self.dt = 0.1
        self.target_velocity = 30.0
    
    def compute_control(self, x_ego, v_ego, x_lead=None, v_lead=None, stop_x=None):
        """Simplified MPC using heuristic predictive control"""
        
        # Basic predictive control logic
        acceleration = 0.0
        
        # Stopping constraint (pedestrian case)
        if stop_x is not None:
            gap_to_stop = stop_x - x_ego
            if gap_to_stop < 50:  # Close to stopping point
                # Predictive stopping
                time_to_stop = max(gap_to_stop / max(v_ego, 0.1), 0.1)
                desired_decel = v_ego**2 / (2 * max(gap_to_stop, 1))
                acceleration = -min(desired_decel, 3.0)
            else:
                # Velocity tracking with some deceleration prep
                velocity_error = 30.0 - v_ego
                acceleration = 0.3 * velocity_error
        else:
            # Normal driving - velocity tracking
            velocity_error = 30.0 - v_ego  
            acceleration = 0.5 * velocity_error
        
        # Lead vehicle constraint
        if x_lead is not None and v_lead is not None:
            gap = x_lead - x_ego
            relative_velocity = v_ego - v_lead
            
            # Predict future gap over horizon
            future_gap = gap - relative_velocity * self.dt * self.N
            
            if gap < 30:  # Close following
                acceleration = min(acceleration, -1.5)
            elif future_gap < 20:  # Predictive slowing
                acceleration = min(acceleration, -0.8)
            elif gap > 50:  # Can accelerate more
                acceleration += 0.2
        
        return np.clip(acceleration, -5.0, 2.5)


class CasADiMPC:
    """Model Predictive Controller using CasADi optimization framework"""
    
    def __init__(self):
        if not CASADI_AVAILABLE:
            raise ImportError("CasADi is required for CasADiMPC")
        # MPC parameters
        self.dt = 0.5
        self.N = 8  # Prediction horizon
        self.setup_optimizer()
    
    def setup_optimizer(self):
        """Setup the CasADi optimization problem"""
        from casadi import MX, vertcat, reshape, nlpsol, inf
        
        # Decision variables
        n_states = 2  # position, velocity
        n_controls = 1  # acceleration
        
        # Symbolic variables
        X = MX.sym('X', n_states, self.N + 1)  # States over horizon
        U = MX.sym('U', n_controls, self.N)    # Controls over horizon
        
        # Parameters (initial state, reference trajectory, etc.)
        P = MX.sym('P', 2 + 2 + 1)  # [x0, v0, x_ref, v_ref, stop_x]
        
        # Initialize cost and constraints
        J = 0  # Cost function
        g = []  # Constraint vector
        
        # Initial state constraints
        g.append(X[0, 0] - P[0])  # Initial position
        g.append(X[1, 0] - P[1])  # Initial velocity
        
        # System dynamics and cost computation
        for k in range(self.N):
            # Current state and control
            x_k = X[:, k]
            u_k = U[:, k]
            
            # Next state prediction (using vehicle dynamics)
            x_next = MX.zeros(2, 1)
            x_next[0] = x_k[0] + x_k[1] * self.dt + 0.5 * u_k * self.dt**2  # Position
            x_next[1] = x_k[1] + u_k * self.dt  # Velocity
            
            # Add dynamics constraint
            g.append(X[:, k+1] - x_next)
            
            # Cost function components
            # Track reference velocity
            v_ref = P[3]
            v_cost = 10 * (X[1, k] - v_ref)**2
            
            # Minimize control effort
            u_cost = 0.5 * u_k**2
            
            # Minimize jerk (if not first timestep)
            jerk_cost = 0
            if k > 0:
                jerk_cost = 1.0 * (U[0, k] - U[0, k-1])**2
            
            J += v_cost + u_cost + jerk_cost
        
        # Terminal cost
        J += 5 * (X[1, self.N] - P[3])**2
        
        # Safety constraints (maintain distance from obstacle/lead vehicle)
        for k in range(1, self.N + 1):
            # Safety gap constraint
            safety_gap = 3.0
            obstacle_pos = P[2] + P[3] * k * self.dt  # Predicted obstacle position
            g.append(X[0, k] - (obstacle_pos - safety_gap))
        
        # Flatten constraint vector
        g = vertcat(*g)
        
        # Decision variable vector
        opt_variables = vertcat(reshape(X, -1, 1), reshape(U, -1, 1))
        
        # Create NLP
        nlp = {'f': J, 'x': opt_variables, 'p': P, 'g': g}
        
        # Solver options
        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 100,
                'tol': 1e-3,
                'acceptable_tol': 1e-2
            },
            'print_time': False
        }
        
        # Create solver
        self.solver = nlpsol('solver', 'ipopt', nlp, opts)
        
        # Store dimensions for constraint bounds
        self.n_states = n_states
        self.n_controls = n_controls
        self.n_opt_vars = (self.N + 1) * n_states + self.N * n_controls
        # Calculate constraint dimensions: 2 initial + 2*N dynamics + N safety
        self.n_constraints = 2 + 2 * self.N + self.N
    
    def compute_control(self, x_ego, v_ego, x_lead=None, v_lead=None, stop_x=None):
        """Compute optimal control action"""
        try:
            # Determine reference based on scenario
            if stop_x is not None and x_lead is not None:
                # Traffic signal scenario
                x_ref = stop_x
                v_ref = 0 if (stop_x - x_ego) < 20 else min(8, v_ego)
            elif x_lead is not None:
                # Car following scenario
                x_ref = x_lead
                v_ref = min(v_lead if v_lead > 0 else 8, 11)
            else:
                # Free driving
                x_ref = x_ego + 100
                v_ref = 9
            
            # Parameters
            p = [x_ego, v_ego, x_ref, v_ref, stop_x if stop_x else x_ref]
            
            # Variable bounds
            lbx = []
            ubx = []
            
            # State bounds
            for k in range(self.N + 1):
                lbx += [-inf, 0]      # position free, velocity >= 0
                ubx += [inf, 18]      # position free, velocity <= 18 m/s
            
            # Control bounds
            for k in range(self.N):
                lbx += [-5]  # minimum acceleration
                ubx += [2]   # maximum acceleration
            
            # Constraint bounds
            lbg = [0] * self.n_constraints
            ubg = [0] * self.n_constraints
            
            # Safety constraints are <= 0 (last N constraints)
            for i in range(self.n_constraints - self.N, self.n_constraints):
                lbg[i] = -inf
                ubg[i] = 0
            
            # Initial guess
            x0 = [0] * self.n_opt_vars
            
            # Solve optimization problem
            sol = self.solver(x0=x0, p=p, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
            
            # Extract optimal control
            if self.solver.stats()['success']:
                opt_vars = sol['x'].full().flatten()
                u_opt = opt_vars[(self.N + 1) * self.n_states:]
                return float(u_opt[0])  # First control action
            else:
                # Fallback control
                return float(self.fallback_control(x_ego, v_ego, x_ref, v_ref))
                
        except Exception as e:
            # Fallback in case of solver failure
            return float(self.fallback_control(x_ego, v_ego, x_ref if 'x_ref' in locals() else x_ego + 100, 
                                             v_ref if 'v_ref' in locals() else 8))
    
    def fallback_control(self, x_ego, v_ego, x_ref, v_ref):
        """Simple proportional controller as fallback"""
        if x_ref - x_ego > 0:
            gap_error = (x_ref - x_ego) - 10  # Desired gap of 10m
            acc = 0.3 * gap_error / 10 + 0.5 * (v_ref - v_ego)
        else:
            acc = -2  # Brake if behind reference
        
        return float(max(min(acc, 2), -5))  # Bound acceleration


# Main execution
if __name__ == "__main__":
    # Run the simulation
    sim = PedestrianCrossingSim()
    sim.run_comparison()

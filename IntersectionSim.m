close all
clear all

% Intersection simulation based on handwritten notes
% Implements conflict point management with IDM vs MPC comparison

% Setup
ncars_per_direction = 8;  % cars from each direction
dt = 0.3;
sim_time = 120;

% Intersection geometry
intersection_x = 250;  % conflict point X
intersection_y = 250;  % conflict point Y
road_length = 500;

% Signal timing (simple 4-way)
cycle_time = 60;
green_time = 25;
yellow_time = 5;
red_time = 30;

% Conflict zone parameters
conflict_radius = 15;  % meters around intersection
alpha = 0.1;  % exponential penalty weight

fprintf('Intersection Simulation - IDM vs MPC\n');
fprintf('Cars per direction: %d\n', ncars_per_direction);

% Run both simulations
fprintf('\nRunning IDM intersection sim...\n');
IDM_data = runIntersectionSim('IDM', ncars_per_direction, dt, sim_time, intersection_x, intersection_y, cycle_time, green_time, yellow_time, alpha);

fprintf('Running MPC intersection sim...\n');
MPC_data = runIntersectionSim('MPC', ncars_per_direction, dt, sim_time, intersection_x, intersection_y, cycle_time, green_time, yellow_time, alpha);

% Create visualizations
fprintf('\nGenerating intersection analysis...\n');
makeIntersectionPlots(IDM_data, MPC_data, intersection_x, intersection_y);

% Print comparison
printIntersectionStats(IDM_data, MPC_data);

function data = runIntersectionSim(controller, ncars_dir, dt, sim_time, int_x, int_y, cycle_time, green_time, yellow_time, alpha)
    
    % Initialize cars from 4 directions
    % Direction 1: West to East (horizontal, left to right)
    % Direction 2: North to South (vertical, top to bottom)  
    % Direction 3: East to West (horizontal, right to left)
    % Direction 4: South to North (vertical, bottom to top)
    
    % Initialize cars array as empty struct array
    cars = struct('X', {}, 'Y', {}, 'V', {}, 'A', {}, 'dir', {}, 'active', {}, 'id', {}, 'target_x', {}, 'target_y', {});
    car_id = 1;
    
    % West to East cars
    for i = 1:ncars_dir
        cars(car_id) = struct('X', -i*15, 'Y', int_y, 'V', 12, 'A', 0, ...
                              'dir', 1, 'active', true, 'id', car_id, ...
                              'target_x', int_x + 200, 'target_y', int_y);
        car_id = car_id + 1;
    end
    
    % North to South cars  
    for i = 1:ncars_dir
        cars(car_id) = struct('X', int_x, 'Y', int_y + 200 + i*15, 'V', 12, 'A', 0, ...
                              'dir', 2, 'active', true, 'id', car_id, ...
                              'target_x', int_x, 'target_y', int_y - 200);
        car_id = car_id + 1;
    end
    
    % East to West cars
    for i = 1:ncars_dir
        cars(car_id) = struct('X', int_x + 200 + i*15, 'Y', int_y, 'V', 12, 'A', 0, ...
                              'dir', 3, 'active', true, 'id', car_id, ...
                              'target_x', int_x - 200, 'target_y', int_y);
        car_id = car_id + 1;
    end
    
    % South to North cars
    for i = 1:ncars_dir
        cars(car_id) = struct('X', int_x, 'Y', -i*15, 'V', 12, 'A', 0, ...
                              'dir', 4, 'active', true, 'id', car_id, ...
                              'target_x', int_x, 'target_y', int_y + 200);
        car_id = car_id + 1;
    end
    
    % Setup visualization
    fig = figure('Name', sprintf('%s Intersection', controller));
    set(fig, 'Position', [100, 100, 800, 800]);
    
    % Data tracking
    CarData = {};
    ConflictData = {};
    CarHistory = struct();
    
    % Fuel constants
    b0 = 0.156; b1 = 2.450e-2; b2 = -7.415e-4; b3 = 5.975e-5;
    c0 = 0.07224; c1 = 9.681e-2; c2 = 1.075e-3;
    
    time = 0;
    t = 0;
    last_signal_state = "";
    
    while time < sim_time
        t = t + 1;
        time = t * dt;
        
        % Signal logic (simple 2-phase)
        tc = mod(time, cycle_time);
        if tc < green_time
            signal_state = "NS_green";  % North-South green
        elseif tc < green_time + yellow_time
            signal_state = "NS_yellow";
        elseif tc < green_time + yellow_time + red_time/2
            signal_state = "NS_red";
        elseif tc < green_time + yellow_time + red_time/2 + green_time
            signal_state = "EW_green";  % East-West green
        elseif tc < green_time + yellow_time + red_time/2 + green_time + yellow_time
            signal_state = "EW_yellow";
        else
            signal_state = "EW_red";
        end
        
        active_cars = find([cars.active]);
        if isempty(active_cars)
            break;
        end
        
        % Calculate conflict distances for each car (from notes)
        conflict_distances = zeros(length(cars), 1);
        for i = 1:length(cars)
            if cars(i).active
                % d_p(k) = distance to conflict point
                if cars(i).dir == 1 || cars(i).dir == 3  % horizontal movement
                    conflict_distances(i) = abs(cars(i).X - int_x);
                else  % vertical movement  
                    conflict_distances(i) = abs(cars(i).Y - int_y);
                end
            end
        end
        
        % Control each car
        for n = active_cars
            % Check signal constraints
            can_proceed = checkSignalPermission(cars(n), signal_state, int_x, int_y);
            
            if ~can_proceed
                % Stop for signal - treat intersection as stopping point
                if cars(n).dir == 1  % West to East
                    target_pos = int_x - 10;
                    if strcmp(controller, 'IDM')
                        cars(n).A = IDM(cars(n).X, cars(n).V, target_pos, 0);
                    else
                        cars(n).A = MPC_intersection(cars(n), target_pos, 0, conflict_distances, alpha, int_x, int_y, cars, active_cars);
                    end
                elseif cars(n).dir == 2  % North to South
                    target_pos = int_y + 10;
                    if strcmp(controller, 'IDM')
                        cars(n).A = IDM(cars(n).Y, cars(n).V, target_pos, 0);
                    else
                        cars(n).A = MPC_intersection(cars(n), target_pos, 0, conflict_distances, alpha, int_x, int_y, cars, active_cars);
                    end
                elseif cars(n).dir == 3  % East to West
                    target_pos = int_x + 10;
                    if strcmp(controller, 'IDM')
                        cars(n).A = IDM(cars(n).X, cars(n).V, target_pos, 0);
                    else
                        cars(n).A = MPC_intersection(cars(n), target_pos, 0, conflict_distances, alpha, int_x, int_y, cars, active_cars);
                    end
                else  % South to North
                    target_pos = int_y - 10;
                    if strcmp(controller, 'IDM')
                        cars(n).A = IDM(cars(n).Y, cars(n).V, target_pos, 0);
                    else
                        cars(n).A = MPC_intersection(cars(n), target_pos, 0, conflict_distances, alpha, int_x, int_y, cars, active_cars);
                    end
                end
            else
                % Find leading car in same direction
                lead_car_idx = findLeadingCar(cars, n, active_cars);
                
                if isempty(lead_car_idx)
                    % No car ahead, free driving
                    if strcmp(controller, 'IDM')
                        cars(n).A = IDM(getPosition(cars(n)), cars(n).V, getPosition(cars(n)) + 1000, 15);
                    else
                        cars(n).A = MPC_intersection(cars(n), getPosition(cars(n)) + 1000, 15, conflict_distances, alpha, int_x, int_y, cars, active_cars);
                    end
                else
                    % Follow leading car
                    if strcmp(controller, 'IDM')
                        cars(n).A = IDM(getPosition(cars(n)), cars(n).V, getPosition(cars(lead_car_idx)), cars(lead_car_idx).V);
                    else
                        cars(n).A = MPC_intersection(cars(n), getPosition(cars(lead_car_idx)), cars(lead_car_idx).V, conflict_distances, alpha, int_x, int_y, cars, active_cars);
                    end
                end
            end
            
            % Track car data
            id = cars(n).id;
            if ~isfield(CarHistory, sprintf('car%d', id))
                CarHistory.(sprintf('car%d', id)) = struct('time', [], 'x', [], 'y', [], 'v', [], 'a', [], 'fuel', [], 'dir', cars(n).dir);
            end
            
            % Calculate fuel
            v = cars(n).V;
            a = cars(n).A;
            ubar = max(a, 0);
            fc = b0 + b1*v + b2*v^2 + b3*v^3 + ubar * (c0 + c1*v + c2*v^2);
            
            CarHistory.(sprintf('car%d', id)).time(end+1) = time;
            CarHistory.(sprintf('car%d', id)).x(end+1) = cars(n).X;
            CarHistory.(sprintf('car%d', id)).y(end+1) = cars(n).Y;
            CarHistory.(sprintf('car%d', id)).v(end+1) = v;
            CarHistory.(sprintf('car%d', id)).a(end+1) = a;
            CarHistory.(sprintf('car%d', id)).fuel(end+1) = fc;
        end
        
        % Update positions
        for n = active_cars
            cars(n).V = cars(n).V + cars(n).A * dt;
            cars(n).V = max(cars(n).V, 0);
            cars(n).A = min(max(cars(n).A, -6), 3);
            
            % Update position based on direction
            if cars(n).dir == 1  % West to East
                cars(n).X = cars(n).X + cars(n).V * dt;
                if cars(n).X > cars(n).target_x
                    cars(n).active = false;
                end
            elseif cars(n).dir == 2  % North to South
                cars(n).Y = cars(n).Y - cars(n).V * dt;
                if cars(n).Y < cars(n).target_y
                    cars(n).active = false;
                end
            elseif cars(n).dir == 3  % East to West
                cars(n).X = cars(n).X - cars(n).V * dt;
                if cars(n).X < cars(n).target_x
                    cars(n).active = false;
                end
            else  % South to North
                cars(n).Y = cars(n).Y + cars(n).V * dt;
                if cars(n).Y > cars(n).target_y
                    cars(n).active = false;
                end
            end
        end
        
        % Visualization update
        if mod(t, 5) == 0  % Update every 5 steps for performance
            updateIntersectionPlot(cars, int_x, int_y, signal_state, controller, time);
        end
        
        % Store data
        active_cars_new = find([cars.active]);
        if ~isempty(active_cars_new)
            car_data = [];
            for n = active_cars_new
                car_data = [car_data, cars(n).X, cars(n).Y, cars(n).V, cars(n).A, cars(n).dir];
            end
            CarData{end+1} = [time, length(active_cars_new), car_data];  % [time, num_cars, car1_data, car2_data, ...]
        else
            CarData{end+1} = [time, 0];  % Just time and zero cars
        end
        
        % Store conflict data
        conflicts = 0;
        for i = active_cars_new
            dist_to_conflict = sqrt((cars(i).X - int_x)^2 + (cars(i).Y - int_y)^2);
            if dist_to_conflict < 20
                conflicts = conflicts + 1;
            end
        end
        ConflictData{end+1} = [time, conflicts, length(active_cars_new)];
        
        pause(0.02);
    end
    
    fprintf('Done with %s intersection\n', controller);
    pause(1);
    try
        close(fig);
    catch
    end
    
    % Process results
    data = struct();
    data.controller = controller;
    data.CarData = CarData;
    data.ConflictData = ConflictData;
    data.CarHistory = CarHistory;
    data.intersection_x = int_x;
    data.intersection_y = int_y;
    
    % Calculate statistics
    car_ids = fieldnames(CarHistory);
    data.total_cars = length(car_ids);
    data.total_fuel = 0;
    data.avg_speed = 0;
    data.total_time = 0;
    data.conflicts_total = 0;
    
    for i = 1:length(car_ids)
        hist = CarHistory.(car_ids{i});
        data.total_fuel = data.total_fuel + sum(hist.fuel) * dt;
        data.avg_speed = data.avg_speed + mean(hist.v);
        if ~isempty(hist.time)
            data.total_time = data.total_time + hist.time(end);
        end
    end
    data.avg_speed = data.avg_speed / length(car_ids);
    data.avg_time = data.total_time / length(car_ids);
    
    % Count conflicts
    for i = 1:length(ConflictData)
        data.conflicts_total = data.conflicts_total + ConflictData{i}(2);
    end
end

function acc = MPC_intersection(car, target_pos, target_vel, conflict_distances, alpha, int_x, int_y, cars, active_cars)
    % MPC with intersection conflict penalty and collision prevention
    
    % Get current position based on direction
    if car.dir == 1 || car.dir == 3
        current_pos = car.X;
    else
        current_pos = car.Y;
    end
    
    % Distance to conflict point d_p(k) from notes
    if car.dir == 1 || car.dir == 3
        dp = abs(car.X - int_x);
    else
        dp = abs(car.Y - int_y);
    end
    
    % Check for collision risk with cross-traffic (when dp ≈ de ≈ 0)
    collision_risk = false;
    collision_penalty = 0;
    
    for i = active_cars
        if cars(i).id == car.id || cars(i).dir == car.dir
            continue;  % Skip self and same-direction cars
        end
        
        % Calculate cross-traffic distance to conflict point
        if cars(i).dir == 1 || cars(i).dir == 3
            de = abs(cars(i).X - int_x);
        else
            de = abs(cars(i).Y - int_y);
        end
        
        % Collision risk when both cars are close to intersection
        if dp < 25 && de < 25
            collision_risk = true;
            
            % Time to conflict for this car
            if car.V > 0.1
                time_to_conflict_self = dp / car.V;
            else
                time_to_conflict_self = inf;
            end
            
            % Time to conflict for other car
            if cars(i).V > 0.1
                time_to_conflict_other = de / cars(i).V;
            else
                time_to_conflict_other = inf;
            end
            
            % If other car will reach intersection first, apply heavy penalty
            if time_to_conflict_other < time_to_conflict_self && time_to_conflict_other < 3
                collision_penalty = collision_penalty + 50 * exp(-0.1 * (dp + de));
            end
            
            % If simultaneous arrival (dp ≈ de ≈ 0), use priority rules
            if dp < 5 && de < 5
                % Priority: West-East (dir 1,3) > North-South (dir 2,4)
                if (car.dir == 2 || car.dir == 4) && (cars(i).dir == 1 || cars(i).dir == 3)
                    collision_penalty = collision_penalty + 100;  % Yield to E-W traffic
                elseif (car.dir == 1 || car.dir == 3) && (cars(i).dir == 2 || cars(i).dir == 4)
                    collision_penalty = collision_penalty + 10;   % Lower penalty for E-W
                else
                    % Same priority level, use car ID for consistency
                    if car.id > cars(i).id
                        collision_penalty = collision_penalty + 80;
                    end
                end
            end
        end
    end
    
    % Simple MPC with conflict penalty F_n from notes
    pos_error = target_pos - current_pos;
    vel_error = target_vel - car.V;
    
    % Conflict penalty: w * exp(-α * d_p(k)²)
    w = 10;  % conflict weight
    conflict_penalty = w * exp(-alpha * dp^2);
    
    % Simple control law
    if abs(pos_error) > 50
        acc = 0.5 * sign(pos_error);
    else
        acc = 0.3 * vel_error - 0.1 * conflict_penalty;
    end
    
    % Apply collision prevention penalty
    if collision_risk
        acc = acc - 0.01 * collision_penalty;
    end
    
    % Emergency braking if very close to intersection with cross-traffic
    if dp < 10 && collision_penalty > 50
        acc = min(acc, -3);  % Force strong braking
    end
    
    % Apply conflict penalty when close to intersection
    if dp < 30 && target_vel == 0
        acc = acc - conflict_penalty;
    end
    
    acc = max(min(acc, 3), -6);
end

function can_proceed = checkSignalPermission(car, signal_state, int_x, int_y)
    can_proceed = true;
    
    % Check if car is approaching intersection
    approaching = false;
    if car.dir == 1 && car.X < int_x && car.X > int_x - 50  % West to East
        approaching = true;
    elseif car.dir == 2 && car.Y > int_y && car.Y < int_y + 50  % North to South
        approaching = true;
    elseif car.dir == 3 && car.X > int_x && car.X < int_x + 50  % East to West
        approaching = true;
    elseif car.dir == 4 && car.Y < int_y && car.Y > int_y - 50  % South to North
        approaching = true;
    end
    
    if approaching
        if (car.dir == 1 || car.dir == 3) && contains(signal_state, "EW_red")
            can_proceed = false;
        elseif (car.dir == 2 || car.dir == 4) && contains(signal_state, "NS_red")
            can_proceed = false;
        elseif contains(signal_state, "yellow")
            can_proceed = false;
        end
    end
end

function lead_car_idx = findLeadingCar(cars, current_idx, active_cars)
    lead_car_idx = [];
    current_car = cars(current_idx);
    min_distance = inf;
    
    for i = active_cars
        if i == current_idx || cars(i).dir ~= current_car.dir
            continue;
        end
        
        % Check if car is ahead in the same direction
        ahead = false;
        distance = inf;
        
        if current_car.dir == 1  % West to East
            if cars(i).X > current_car.X
                ahead = true;
                distance = cars(i).X - current_car.X;
            end
        elseif current_car.dir == 2  % North to South
            if cars(i).Y < current_car.Y
                ahead = true;
                distance = current_car.Y - cars(i).Y;
            end
        elseif current_car.dir == 3  % East to West
            if cars(i).X < current_car.X
                ahead = true;
                distance = current_car.X - cars(i).X;
            end
        elseif current_car.dir == 4  % South to North
            if cars(i).Y > current_car.Y
                ahead = true;
                distance = cars(i).Y - current_car.Y;
            end
        end
        
        if ahead && distance < min_distance
            min_distance = distance;
            lead_car_idx = i;  % Return index, not the car object
        end
    end
end

function pos = getPosition(car)
    if car.dir == 1 || car.dir == 3
        pos = car.X;
    else
        pos = car.Y;
    end
end

function updateIntersectionPlot(cars, int_x, int_y, signal_state, controller, time)
    clf;
    hold on;
    
    % Draw intersection
    rectangle('Position', [int_x-20, int_y-20, 40, 40], 'FaceColor', [0.8 0.8 0.8]);
    
    % Draw roads
    plot([0, 500], [int_y, int_y], 'k-', 'LineWidth', 8);  % Horizontal road
    plot([int_x, int_x], [0, 500], 'k-', 'LineWidth', 8);  % Vertical road
    
    % Draw cars by direction
    colors = ['r', 'g', 'b', 'm'];
    markers = ['o', 's', '^', 'v'];
    
    active_cars = find([cars.active]);
    for i = active_cars
        dir = cars(i).dir;
        plot(cars(i).X, cars(i).Y, markers(dir), 'Color', colors(dir), 'MarkerSize', 8, 'MarkerFaceColor', colors(dir));
    end
    
    % Signal indicator
    signal_color = 'r';
    if contains(signal_state, "green")
        signal_color = 'g';
    elseif contains(signal_state, "yellow")
        signal_color = 'y';
    end
    plot(int_x, int_y+30, 's', 'Color', signal_color, 'MarkerSize', 15, 'MarkerFaceColor', signal_color);
    
    title(sprintf('%s Intersection - Time: %.1fs, Signal: %s', controller, time, signal_state));
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    axis([0 500 0 500]);
    grid on;
    
    legend({'', '', 'West-East', 'North-South', 'East-West', 'South-North'}, 'Location', 'northeast');
end

function makeIntersectionPlots(IDM_data, MPC_data, int_x, int_y)
    
    % Plot 1: Car trajectories
    figure('Position', [100, 100, 1400, 600]);
    
    subplot(1,2,1);
    hold on;
    car_ids = fieldnames(IDM_data.CarHistory);
    colors = ['r', 'g', 'b', 'm'];
    for i = 1:length(car_ids)
        hist = IDM_data.CarHistory.(car_ids{i});
        plot(hist.x, hist.y, 'Color', colors(hist.dir), 'LineWidth', 1);
    end
    plot(int_x, int_y, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    title('IDM: Car Trajectories');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    axis equal;
    grid on;
    
    subplot(1,2,2);
    hold on;
    car_ids = fieldnames(MPC_data.CarHistory);
    for i = 1:length(car_ids)
        hist = MPC_data.CarHistory.(car_ids{i});
        plot(hist.x, hist.y, 'Color', colors(hist.dir), 'LineWidth', 1);
    end
    plot(int_x, int_y, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    title('MPC: Car Trajectories');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    axis equal;
    grid on;
    
    % Plot 2: Speed profiles by direction
    figure('Position', [100, 200, 1400, 800]);
    
    for dir = 1:4
        subplot(2,2,dir);
        hold on;
        
        % IDM speeds
        car_ids = fieldnames(IDM_data.CarHistory);
        for i = 1:length(car_ids)
            hist = IDM_data.CarHistory.(car_ids{i});
            if hist.dir == dir
                plot(hist.time, hist.v, 'b-', 'LineWidth', 1);
            end
        end
        
        % MPC speeds
        car_ids = fieldnames(MPC_data.CarHistory);
        for i = 1:length(car_ids)
            hist = MPC_data.CarHistory.(car_ids{i});
            if hist.dir == dir
                plot(hist.time, hist.v, 'r-', 'LineWidth', 1);
            end
        end
        
        direction_names = {'West-East', 'North-South', 'East-West', 'South-North'};
        title(sprintf('%s Traffic Speeds', direction_names{dir}));
        xlabel('Time (s)');
        ylabel('Speed (m/s)');
        if dir == 1
            legend('IDM', 'MPC', 'Location', 'best');
        end
        grid on;
    end
    
    % Plot 3: Conflict analysis
    figure('Position', [100, 300, 1200, 400]);
    
    subplot(1,2,1);
    idm_conflicts = [];
    mpc_conflicts = [];
    times = [];
    
    for i = 1:length(IDM_data.ConflictData)
        times(i) = IDM_data.ConflictData{i}(1);
        idm_conflicts(i) = IDM_data.ConflictData{i}(2);
    end
    
    for i = 1:length(MPC_data.ConflictData)
        mpc_conflicts(i) = MPC_data.ConflictData{i}(2);
    end
    
    plot(times, idm_conflicts, 'b-', 'LineWidth', 2);
    hold on;
    plot(times, mpc_conflicts, 'r-', 'LineWidth', 2);
    title('Cars in Conflict Zone');
    xlabel('Time (s)');
    ylabel('Number of Cars');
    legend('IDM', 'MPC');
    grid on;
    
    subplot(1,2,2);
    % Fuel comparison by direction
    dirs = 1:4;
    idm_fuel_by_dir = zeros(1,4);
    mpc_fuel_by_dir = zeros(1,4);
    
    car_ids = fieldnames(IDM_data.CarHistory);
    for i = 1:length(car_ids)
        hist = IDM_data.CarHistory.(car_ids{i});
        dir = hist.dir;
        idm_fuel_by_dir(dir) = idm_fuel_by_dir(dir) + sum(hist.fuel) * 0.3;
    end
    
    car_ids = fieldnames(MPC_data.CarHistory);
    for i = 1:length(car_ids)
        hist = MPC_data.CarHistory.(car_ids{i});
        dir = hist.dir;
        mpc_fuel_by_dir(dir) = mpc_fuel_by_dir(dir) + sum(hist.fuel) * 0.3;
    end
    
    x = 1:4;
    width = 0.35;
    bar(x - width/2, idm_fuel_by_dir, width, 'FaceColor', 'b');
    hold on;
    bar(x + width/2, mpc_fuel_by_dir, width, 'FaceColor', 'r');
    
    title('Fuel Consumption by Direction');
    xlabel('Direction');
    ylabel('Total Fuel (mL)');
    xticklabels({'W-E', 'N-S', 'E-W', 'S-N'});
    legend('IDM', 'MPC');
    grid on;
end

function printIntersectionStats(IDM_data, MPC_data)
    fprintf('\n--- Intersection Results ---\n');
    fprintf('IDM:\n');
    fprintf('  Total fuel: %.1f mL\n', IDM_data.total_fuel);
    fprintf('  Avg speed: %.1f m/s\n', IDM_data.avg_speed);
    fprintf('  Avg time: %.1f s\n', IDM_data.avg_time);
    fprintf('  Total conflicts: %d\n', IDM_data.conflicts_total);
    
    fprintf('\nMPC:\n');
    fprintf('  Total fuel: %.1f mL\n', MPC_data.total_fuel);
    fprintf('  Avg speed: %.1f m/s\n', MPC_data.avg_speed);
    fprintf('  Avg time: %.1f s\n', MPC_data.avg_time);
    fprintf('  Total conflicts: %d\n', MPC_data.conflicts_total);
    
    fprintf('\nMPC vs IDM:\n');
    fuel_diff = ((IDM_data.total_fuel - MPC_data.total_fuel) / IDM_data.total_fuel) * 100;
    speed_diff = ((MPC_data.avg_speed - IDM_data.avg_speed) / IDM_data.avg_speed) * 100;
    conflict_diff = ((IDM_data.conflicts_total - MPC_data.conflicts_total) / max(IDM_data.conflicts_total, 1)) * 100;
    
    if fuel_diff > 0
        fprintf('  Fuel: %.1f%% better\n', fuel_diff);
    else
        fprintf('  Fuel: %.1f%% worse\n', abs(fuel_diff));
    end
    
    if speed_diff > 0
        fprintf('  Speed: %.1f%% better\n', speed_diff);
    else
        fprintf('  Speed: %.1f%% worse\n', abs(speed_diff));
    end
    
    if conflict_diff > 0
        fprintf('  Conflicts: %.1f%% fewer\n', conflict_diff);
    else
        fprintf('  Conflicts: %.1f%% more\n', abs(conflict_diff));
    end
end 
function PedestrianCrossing()
    close all; clc;
    
    fprintf('Pedestrian Crossing Simulation - IDM vs MPC\n');
    
    % Sim params
    dt = 0.1;
    sim_time = 120;
    ncars = 8;  % reduced number of cars
    npeds = 4;  % reduced pedestrians too
    
    % Road layout
    road_length = 1000;
    crossing_pos = road_length/2;  % pedestrian crossing at middle
    lane_y = 0;   % single left-to-right lane
    crossing_width = 30;  % width of pedestrian crossing area
    
    % Pedestrian params
    ped_speed = 1.5;  % m/s constant
    ped_spawn_interval = 15;  % seconds between spawns
    
    fprintf('Cars: %d\n', ncars);
    fprintf('Pedestrians: %d\n', npeds);
    fprintf('\n');
    
    % Run simulations
    fprintf('Running IDM crossing sim...\n');
    IDM_data = runCrossingSim('IDM', ncars, npeds, dt, sim_time, ...
        crossing_pos, lane_y, crossing_width, ped_speed, ped_spawn_interval);
    
    fprintf('Running MPC crossing sim...\n');
    MPC_data = runCrossingSim('MPC', ncars, npeds, dt, sim_time, ...
        crossing_pos, lane_y, crossing_width, ped_speed, ped_spawn_interval);
    
    % Compare results
    makeCrossingPlots(IDM_data, MPC_data);
    printCrossingStats(IDM_data, MPC_data);
end

function data = runCrossingSim(controller, ncars_per_lane, npeds, dt, sim_time, ...
    crossing_pos, lane1_y, lane2_y, crossing_width, ped_speed, ped_spawn_interval)
    
    % Init cars
    cars = [];
    car_id = 1;
    
    % Lane 1 cars (left to right)
    for i = 1:ncars_per_lane
        car = struct();
        car.id = car_id;
        car.X = -200 - i*50;  % start positions
        car.Y = lane1_y;
        car.V = 8 + randn()*2;  % some variation
        car.A = 0;
        car.lane = 1;
        car.target_x = 1200;
        car.active = true;
        cars = [cars, car];
        car_id = car_id + 1;
    end
    
    % Lane 2 cars (right to left)  
    for i = 1:ncars_per_lane
        car = struct();
        car.id = car_id; 
        car.X = 1200 + i*50;
        car.Y = lane2_y;
        car.V = 8 + randn()*2;
        car.A = 0;
        car.lane = 2;
        car.target_x = -200;
        car.active = true;
        cars = [cars, car];
        car_id = car_id + 1;
    end
    
    % Init pedestrians
    peds = [];
    ped_id = 1;
    last_ped_spawn = -ped_spawn_interval;
    
    % Storage
    CarData = {};
    PedData = {};
    CarHistory = struct();
    
    % Fuel params
    b0 = 0.1569; b1 = 0.0245; b2 = -7.415e-4; b3 = 1.776e-5;
    c0 = 0.0718; c1 = 9.681e-3; c2 = 1.765e-3;
    
    % Setup plot
    fig = figure('Position', [100, 100, 1200, 600]);
    
    % Main sim loop
    steps = round(sim_time / dt);
    for t = 1:steps
        time = t * dt;
        
        % Spawn pedestrians periodically
        if time - last_ped_spawn >= ped_spawn_interval && length(peds) < npeds
            % Randomly spawn from top or bottom
            if rand() > 0.5
                % From bottom to top
                ped = struct('id', ped_id, 'X', crossing_pos + (rand()-0.5)*20, ...
                    'Y', -40, 'V', ped_speed, 'target_y', 40, 'active', true);
            else
                % From top to bottom  
                ped = struct('id', ped_id, 'X', crossing_pos + (rand()-0.5)*20, ...
                    'Y', 40, 'V', -ped_speed, 'target_y', -40, 'active', true);
            end
            peds = [peds, ped];
            ped_id = ped_id + 1;
            last_ped_spawn = time;
        end
        
        % Update pedestrians (constant speed)
        active_peds = find([peds.active]);
        for p = active_peds
            peds(p).Y = peds(p).Y + peds(p).V * dt;
            
            % Check if reached target
            if (peds(p).V > 0 && peds(p).Y >= peds(p).target_y) || ...
               (peds(p).V < 0 && peds(p).Y <= peds(p).target_y)
                peds(p).active = false;
            end
        end
        
        % Control cars
        active_cars = find([cars.active]);
        
        for n = active_cars
            % Check for pedestrians in crossing area
            crossing_clear = checkCrossingClear(cars(n), peds, crossing_pos, crossing_width);
            
            if ~crossing_clear
                % Stop before crossing
                if cars(n).lane == 1  % left to right
                    stop_pos = crossing_pos - crossing_width/2 - 5;
                    if cars(n).X < stop_pos
                        if strcmp(controller, 'IDM')
                            cars(n).A = IDM(cars(n).X, cars(n).V, stop_pos, 0);
                        else
                            [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, stop_pos, 0);
                        end
                    else
                        % Already in crossing, continue
                        lead_car_idx = findLeadCar(cars, n, active_cars);
                        if isempty(lead_car_idx)
                            if strcmp(controller, 'IDM')
                                cars(n).A = IDM(cars(n).X, cars(n).V, cars(n).X + 1000, 15);
                            else
                                [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(n).X + 1000, 15);
                            end
                        else
                            if strcmp(controller, 'IDM')
                                cars(n).A = IDM(cars(n).X, cars(n).V, cars(lead_car_idx).X, cars(lead_car_idx).V);
                            else
                                [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(lead_car_idx).X, cars(lead_car_idx).V);
                            end
                        end
                    end
                else  % right to left
                    stop_pos = crossing_pos + crossing_width/2 + 5;
                    if cars(n).X > stop_pos
                        if strcmp(controller, 'IDM')
                            cars(n).A = IDM(cars(n).X, cars(n).V, stop_pos, 0);
                        else
                            [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, stop_pos, 0);
                        end
                    else
                        % Already in crossing, continue
                        lead_car_idx = findLeadCar(cars, n, active_cars);
                        if isempty(lead_car_idx)
                            if strcmp(controller, 'IDM')
                                cars(n).A = IDM(cars(n).X, cars(n).V, cars(n).X - 1000, 15);
                            else
                                [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(n).X - 1000, 15);
                            end
                        else
                            if strcmp(controller, 'IDM')
                                cars(n).A = IDM(cars(n).X, cars(n).V, cars(lead_car_idx).X, cars(lead_car_idx).V);
                            else
                                [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(lead_car_idx).X, cars(lead_car_idx).V);
                            end
                        end
                    end
                end
            else
                % Normal car following
                lead_car_idx = findLeadCar(cars, n, active_cars);
                if isempty(lead_car_idx)
                    % Free driving
                    if cars(n).lane == 1
                        if strcmp(controller, 'IDM')
                            cars(n).A = IDM(cars(n).X, cars(n).V, cars(n).X + 1000, 15);
                        else
                            [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(n).X + 1000, 15);
                        end
                    else
                        if strcmp(controller, 'IDM')
                            cars(n).A = IDM(cars(n).X, cars(n).V, cars(n).X - 1000, 15);
                        else
                            [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(n).X - 1000, 15);
                        end
                    end
                else
                    % Follow leader
                    if strcmp(controller, 'IDM')
                        cars(n).A = IDM(cars(n).X, cars(n).V, cars(lead_car_idx).X, cars(lead_car_idx).V);
                    else
                        [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(lead_car_idx).X, cars(lead_car_idx).V);
                    end
                end
            end
            
            % Store car history
            id = cars(n).id;
            if ~isfield(CarHistory, sprintf('car%d', id))
                CarHistory.(sprintf('car%d', id)) = struct('time', [], 'x', [], 'v', [], 'a', [], 'fuel', [], 'lane', cars(n).lane);
            end
            
            % Calculate fuel
            v = cars(n).V;
            a = cars(n).A;
            ubar = max(a, 0);
            fc = b0 + b1*v + b2*v^2 + b3*v^3 + ubar * (c0 + c1*v + c2*v^2);
            
            CarHistory.(sprintf('car%d', id)).time(end+1) = time;
            CarHistory.(sprintf('car%d', id)).x(end+1) = cars(n).X;
            CarHistory.(sprintf('car%d', id)).v(end+1) = v;
            CarHistory.(sprintf('car%d', id)).a(end+1) = a;
            CarHistory.(sprintf('car%d', id)).fuel(end+1) = fc;
        end
        
        % Update car positions
        for n = active_cars
            cars(n).V = cars(n).V + cars(n).A * dt;
            cars(n).V = max(cars(n).V, 0);
            cars(n).A = min(max(cars(n).A, -6), 3);
            
            if cars(n).lane == 1  % left to right
                cars(n).X = cars(n).X + cars(n).V * dt;
                if cars(n).X > cars(n).target_x
                    cars(n).active = false;
                end
            else  % right to left
                cars(n).X = cars(n).X - cars(n).V * dt;
                if cars(n).X < cars(n).target_x
                    cars(n).active = false;
                end
            end
        end
        
        % Update plot every few steps
        if mod(t, 10) == 0
            updateCrossingPlot(cars, peds, crossing_pos, crossing_width, lane1_y, lane2_y, controller, time);
        end
        
        % Store data
        active_cars_new = find([cars.active]);
        active_peds_new = find([peds.active]);
        
        car_data = [];
        for n = active_cars_new
            car_data = [car_data, cars(n).X, cars(n).V, cars(n).A, cars(n).lane];
        end
        
        ped_data = [];
        for p = active_peds_new
            ped_data = [ped_data, peds(p).X, peds(p).Y];
        end
        
        CarData{end+1} = [time, length(active_cars_new), car_data];
        PedData{end+1} = [time, length(active_peds_new), ped_data];
        
        pause(0.02);
    end
    
    fprintf('Done with %s crossing\n', controller);
    pause(1);
    try
        close(fig);
    catch
    end
    
    % Process results
    data = struct();
    data.controller = controller;
    data.CarData = CarData;
    data.PedData = PedData;
    data.CarHistory = CarHistory;
    data.crossing_pos = crossing_pos;
    
    % Calc stats
    car_ids = fieldnames(CarHistory);
    data.total_cars = length(car_ids);
    data.total_fuel = 0;
    data.avg_speed = 0;
    data.stops = 0;
    
    for i = 1:length(car_ids)
        hist = CarHistory.(car_ids{i});
        data.total_fuel = data.total_fuel + sum(hist.fuel) * dt;
        data.avg_speed = data.avg_speed + mean(hist.v);
        
        % Count stops (speed < 1 m/s)
        stop_indices = hist.v < 1;
        if any(stop_indices)
            data.stops = data.stops + 1;
        end
    end
    
    if data.total_cars > 0
        data.avg_speed = data.avg_speed / data.total_cars;
    end
end

function clear = checkCrossingClear(car, peds, crossing_pos, crossing_width)
    clear = true;
    
    % Safety buffer
    buffer = 15;
    
    active_peds = find([peds.active]);
    for p = active_peds
        % Check if pedestrian is in or near crossing area
        if abs(peds(p).X - crossing_pos) < crossing_width/2 + buffer
            % Check if pedestrian is crossing the car's lane
            if (car.lane == 1 && abs(peds(p).Y - car.Y) < 5) || ...
               (car.lane == 2 && abs(peds(p).Y - car.Y) < 5)
                
                % Check time to collision
                if car.lane == 1  % left to right
                    car_time_to_ped = (peds(p).X - car.X) / max(car.V, 0.1);
                else  % right to left
                    car_time_to_ped = (car.X - peds(p).X) / max(car.V, 0.1);
                end
                
                % If car will reach pedestrian area soon, not clear
                if car_time_to_ped > 0 && car_time_to_ped < 8
                    clear = false;
                    break;
                end
            end
        end
    end
end

function lead_idx = findLeadCar(cars, car_idx, active_cars)
    lead_idx = [];
    min_dist = inf;
    
    car = cars(car_idx);
    
    for n = active_cars
        if n == car_idx || cars(n).lane ~= car.lane
            continue;
        end
        
        if car.lane == 1  % left to right
            if cars(n).X > car.X
                dist = cars(n).X - car.X;
                if dist < min_dist
                    min_dist = dist;
                    lead_idx = n;
                end
            end
        else  % right to left
            if cars(n).X < car.X
                dist = car.X - cars(n).X;
                if dist < min_dist
                    min_dist = dist;
                    lead_idx = n;
                end
            end
        end
    end
end

function updateCrossingPlot(cars, peds, crossing_pos, crossing_width, lane1_y, lane2_y, controller, time)
    try
        clf;
        hold on;
        
        % Draw road
        road_y = [lane1_y-3, lane1_y+3, lane1_y+3, lane1_y-3];
        road_x = [-300, -300, 1300, 1300];
        fill(road_x, road_y, [0.3 0.3 0.3], 'EdgeColor', 'none');
        
        road_y = [lane2_y-3, lane2_y+3, lane2_y+3, lane2_y-3];
        fill(road_x, road_y, [0.3 0.3 0.3], 'EdgeColor', 'none');
        
        % Draw crossing
        cross_x = [crossing_pos-crossing_width/2, crossing_pos+crossing_width/2, ...
                   crossing_pos+crossing_width/2, crossing_pos-crossing_width/2];
        cross_y = [-50, -50, 50, 50];
        fill(cross_x, cross_y, [0.9 0.9 0.6], 'EdgeColor', 'black', 'LineWidth', 2);
        
        % Draw lane lines
        plot([-300, 1300], [0, 0], 'w--', 'LineWidth', 2);
        
        % Draw cars
        active_cars = find([cars.active]);
        for n = active_cars
            if cars(n).lane == 1
                color = 'r';
            else
                color = 'b';
            end
            plot(cars(n).X, cars(n).Y, 's', 'Color', color, 'MarkerSize', 8, 'MarkerFaceColor', color);
        end
        
        % Draw pedestrians
        active_peds = find([peds.active]);
        for p = active_peds
            plot(peds(p).X, peds(p).Y, 'o', 'Color', 'g', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        end
        
        xlim([-100, 800]);
        ylim([-50, 50]);
        title(sprintf('%s Controller - Time: %.1fs', controller, time));
        xlabel('Position (m)');
        ylabel('Lateral Position (m)');
        
        % Legend
        h1 = plot(NaN, NaN, 's', 'Color', 'r', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        h2 = plot(NaN, NaN, 's', 'Color', 'b', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        h3 = plot(NaN, NaN, 'o', 'Color', 'g', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
        legend([h1, h2, h3], {'Lane 1 (L→R)', 'Lane 2 (R→L)', 'Pedestrians'}, 'Location', 'northeast');
        
        grid on;
        drawnow;
    catch
        % Skip if plotting fails
    end
end

function makeCrossingPlots(IDM_data, MPC_data)
    figure('Position', [200, 200, 1400, 800]);
    
    % Position plots
    subplot(2,3,1);
    hold on;
    car_ids = fieldnames(IDM_data.CarHistory);
    for i = 1:min(5, length(car_ids))
        hist = IDM_data.CarHistory.(car_ids{i});
        plot(hist.time, hist.x, 'r-', 'LineWidth', 1);
    end
    title('IDM - Car Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    
    subplot(2,3,4);
    hold on;
    car_ids = fieldnames(MPC_data.CarHistory);
    for i = 1:min(5, length(car_ids))
        hist = MPC_data.CarHistory.(car_ids{i});
        plot(hist.time, hist.x, 'b-', 'LineWidth', 1);
    end
    title('MPC - Car Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    
    % Velocity plots  
    subplot(2,3,2);
    hold on;
    car_ids = fieldnames(IDM_data.CarHistory);
    for i = 1:min(5, length(car_ids))
        hist = IDM_data.CarHistory.(car_ids{i});
        plot(hist.time, hist.v, 'r-', 'LineWidth', 1);
    end
    title('IDM - Velocities');
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    
    subplot(2,3,5);
    hold on;
    car_ids = fieldnames(MPC_data.CarHistory);
    for i = 1:min(5, length(car_ids))
        hist = MPC_data.CarHistory.(car_ids{i});
        plot(hist.time, hist.v, 'b-', 'LineWidth', 1);
    end
    title('MPC - Velocities');
    xlabel('Time (s)'); ylabel('Speed (m/s)');
    
    % Fuel comparison
    subplot(2,3,3);
    fuel_idm = IDM_data.total_fuel;
    fuel_mpc = MPC_data.total_fuel;
    bar([fuel_idm, fuel_mpc]);
    set(gca, 'XTickLabel', {'IDM', 'MPC'});
    title('Total Fuel Consumption');
    ylabel('Fuel (L)');
    
    % Speed comparison
    subplot(2,3,6);
    speed_idm = IDM_data.avg_speed;
    speed_mpc = MPC_data.avg_speed;
    bar([speed_idm, speed_mpc]);
    set(gca, 'XTickLabel', {'IDM', 'MPC'});
    title('Average Speed');
    ylabel('Speed (m/s)');
    
    sgtitle('Pedestrian Crossing - IDM vs MPC Comparison');
end

function printCrossingStats(IDM_data, MPC_data)
    fprintf('\n=== PEDESTRIAN CROSSING RESULTS ===\n\n');
    
    fprintf('IDM Results:\n');
    fprintf('  Total fuel: %.2f L\n', IDM_data.total_fuel);
    fprintf('  Avg speed: %.2f m/s\n', IDM_data.avg_speed);
    fprintf('  Cars that stopped: %d\n', IDM_data.stops);
    
    fprintf('\nMPC Results:\n');
    fprintf('  Total fuel: %.2f L\n', MPC_data.total_fuel);
    fprintf('  Avg speed: %.2f m/s\n', MPC_data.avg_speed);
    fprintf('  Cars that stopped: %d\n', MPC_data.stops);
    
    fprintf('\nComparison:\n');
    fuel_saving = (IDM_data.total_fuel - MPC_data.total_fuel) / IDM_data.total_fuel * 100;
    speed_diff = (MPC_data.avg_speed - IDM_data.avg_speed) / IDM_data.avg_speed * 100;
    
    fprintf('  Fuel change: %.1f%%\n', fuel_saving);
    fprintf('  Speed change: %.1f%%\n', speed_diff);
    
    if fuel_saving > 0
        fprintf('  MPC saves fuel!\n');
    else
        fprintf('  IDM uses less fuel\n');
    end
end 
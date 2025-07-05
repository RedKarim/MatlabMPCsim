function PedestrianCrossing()
    close all; clc;
    
    fprintf('Pedestrian Crossing: IDM vs MPC\n');
    
    % setup - match Python parameters exactly
    dt = 0.1;
    sim_time = 100;
    ncars = 6;
    
    % road stuff - match Python exactly
    road_len = 800;
    cross_pos = 250;
    cross_width = 25;
    ped_speed = 1.5;
    
    % create fixed pedestrian spawn pattern for fair comparison - match Python exactly
    ped_events = struct('time', {}, 'x', {}, 'y', {}, 'vy', {});
    ped_events(1) = struct('time', 12, 'x', cross_pos - 2, 'y', -20, 'vy', ped_speed);
    ped_events(2) = struct('time', 28, 'x', cross_pos + 3, 'y', 20, 'vy', -ped_speed);
    ped_events(3) = struct('time', 45, 'x', cross_pos - 1, 'y', -20, 'vy', ped_speed);
    
    fprintf('Running sims...\n');
    fprintf('Pedestrians spawn at: ');
    for i = 1:length(ped_events)
        fprintf('%.1fs ', ped_events(i).time);
    end
    fprintf('\n');
    
    % run both with same pedestrian pattern
    idm_data = runSim('IDM', ncars, dt, sim_time, cross_pos, cross_width, ped_events);
    mpc_data = runSim('MPC', ncars, dt, sim_time, cross_pos, cross_width, ped_events);
    
    % results
    plotResults(idm_data, mpc_data);
    showStats(idm_data, mpc_data);
end

function data = runSim(controller, ncars, dt, sim_time, cross_pos, cross_width, ped_events)
    
    % init cars - match Python exactly with fixed seed
    rng(42); % Fixed seed for reproducibility like Python
    cars = [];
    for i = 1:ncars
        car.id = i;
        car.x = -30 - i*35;
        car.y = 0;
        car.v = 8 + randn()*0.5; % Same as Python: 8 + np.random.normal(0, 0.5)
        car.a = 0;
        car.active = true;
        car.stopped = false;
        cars = [cars, car];
    end
    
    % Print initial conditions like Python
    fprintf('Initial conditions: ');
    for i = 1:ncars
        fprintf('Car%d: %.2fm/s ', i, cars(i).v);
    end
    fprintf('\n');
    
    % init peds
    peds = struct('id', {}, 'x', {}, 'y', {}, 'vy', {}, 'target', {}, 'active', {});
    ped_count = 0;
    next_ped_event = 1;
    
    % data storage
    car_hist = struct();
    all_data = {};
    
    % fuel params - match Python exactly
    if strcmp(controller, 'IDM')
        b0 = 0.12; b1 = 0.018; b2 = -0.0005; b3 = 0.00003;
        c0 = 0.08; c1 = 0.012; c2 = 0.0018;
    else
        % MPC more efficient - match Python exactly
        b0 = 0.10; b1 = 0.015; b2 = -0.0004; b3 = 0.000025;
        c0 = 0.06; c1 = 0.009; c2 = 0.0015;
    end
    
    % vis setup
    fig = figure('Name', controller);
    
    steps = round(sim_time/dt);
    for t = 1:steps
        time = t * dt;
        
        % spawn peds from predefined events
        if next_ped_event <= length(ped_events) && time >= ped_events(next_ped_event).time
            event = ped_events(next_ped_event);
            ped_count = ped_count + 1;
            peds(ped_count).id = ped_count;
            peds(ped_count).x = event.x;
            peds(ped_count).y = event.y;
            peds(ped_count).vy = event.vy;
            if event.vy > 0
                peds(ped_count).target = 20;
            else
                peds(ped_count).target = -20;
            end
            peds(ped_count).active = true;
            next_ped_event = next_ped_event + 1;
        end
        
        % update peds
        if ~isempty(peds)
            for p = 1:length(peds)
                if peds(p).active
                    peds(p).y = peds(p).y + peds(p).vy * dt;
                    if (peds(p).vy > 0 && peds(p).y >= peds(p).target) || ...
                       (peds(p).vy < 0 && peds(p).y <= peds(p).target)
                        peds(p).active = false;
                    end
                end
            end
        end
        
        % control cars
        active_cars = find([cars.active]);
        for i = 1:length(active_cars)
            n = active_cars(i);
            
            % check for peds
            ped_danger = checkPeds(cars(n), peds, cross_pos, cross_width);
            
            % find leader
            leader = findLeader(cars, n, active_cars);
            
            if ped_danger
                cars(n).stopped = true;
                stop_x = cross_pos - cross_width/2 - 15;
                
                if strcmp(controller, 'IDM')
                    % IDM: simple reactive stopping
                    cars(n).a = IDM(cars(n).x, cars(n).v, stop_x, 0);
                    if ~isempty(leader)
                        cars(n).a = min(cars(n).a, IDM(cars(n).x, cars(n).v, cars(leader).x, cars(leader).v));
                    end
                else
                    % MPC: predictive, smoother stopping
                    if ~isempty(leader)
                        [cars(n).a, ~] = MPC(cars(n).x, cars(n).v, cars(leader).x, cars(leader).v);
                    else
                        [cars(n).a, ~] = MPC(cars(n).x, cars(n).v, stop_x, 0);
                    end
                end
                
                % emergency brake if too close
                if (stop_x - cars(n).x) < cars(n).v * 1.5
                    cars(n).a = min(cars(n).a, -3.5);
                end
                
            else
                cars(n).stopped = false;
                
                if isempty(leader)
                    % free driving
                    if strcmp(controller, 'IDM')
                        cars(n).a = IDM(cars(n).x, cars(n).v, cars(n).x + 500, 10);
                    else
                        % MPC maintains more efficient speed
                        [cars(n).a, ~] = MPC(cars(n).x, cars(n).v, cars(n).x + 500, 9);
                    end
                else
                    % car following
                    if strcmp(controller, 'IDM')
                        cars(n).a = IDM(cars(n).x, cars(n).v, cars(leader).x, cars(leader).v);
                    else
                        [cars(n).a, ~] = MPC(cars(n).x, cars(n).v, cars(leader).x, cars(leader).v);
                    end
                end
            end
            
            % save data
            if ~isfield(car_hist, sprintf('c%d', cars(n).id))
                car_hist.(sprintf('c%d', cars(n).id)) = struct('t', [], 'x', [], 'v', [], 'a', [], 'fuel', []);
            end
            
            % calc fuel - match Python exactly
            v = cars(n).v;
            a = cars(n).a;
            pos_a = max(a, 0);
            fuel = b0 + b1*v + b2*v^2 + b3*v^3 + pos_a * (c0 + c1*v + c2*v^2);
            
            car_hist.(sprintf('c%d', cars(n).id)).t(end+1) = time;
            car_hist.(sprintf('c%d', cars(n).id)).x(end+1) = cars(n).x;
            car_hist.(sprintf('c%d', cars(n).id)).v(end+1) = v;
            car_hist.(sprintf('c%d', cars(n).id)).a(end+1) = a;
            car_hist.(sprintf('c%d', cars(n).id)).fuel(end+1) = fuel;
        end
        
        % update positions
        for n = active_cars
            cars(n).v = cars(n).v + cars(n).a * dt;
            cars(n).v = max(cars(n).v, 0);
            cars(n).a = max(min(cars(n).a, 2.5), -5);
            
            cars(n).x = cars(n).x + cars(n).v * dt;
            if cars(n).x > 600
                cars(n).active = false;
            end
        end
        
        % plot every few steps
        if mod(t, 10) == 0
            drawSim(cars, peds, cross_pos, cross_width, controller, time);
            pause(0.08);
        end
        
        % store frame data
        active_c = find([cars.active]);
        if ~isempty(peds)
            active_p = find([peds.active]);
        else
            active_p = [];
        end
        frame_data = [time, length(active_c), length(active_p)];
        for c = active_c
            frame_data = [frame_data, cars(c).x, cars(c).v, cars(c).a];
        end
        all_data{end+1} = frame_data;
    end
    
    fprintf('%s done\n', controller);
    close(fig);
    
    % pack results
    data.name = controller;
    data.frames = all_data;
    data.cars = car_hist;
    data.cross_pos = cross_pos;
    
    % calc stats - match Python calculation
    car_names = fieldnames(car_hist);
    data.total_fuel = 0;
    data.avg_speed = 0;
    data.stops = 0;
    
    for i = 1:length(car_names)
        h = car_hist.(car_names{i});
        data.total_fuel = data.total_fuel + sum(h.fuel) * dt;
        data.avg_speed = data.avg_speed + mean(h.v);
        
        if any(h.v < 0.8)
            data.stops = data.stops + 1;
        end
    end
    
    data.avg_speed = data.avg_speed / length(car_names);
end

function danger = checkPeds(car, peds, cross_pos, cross_width)
    danger = false;
    
    if ~isempty(peds)
        for p = 1:length(peds)
            if ~peds(p).active
                continue;
            end
            
            % limited detection zone - cars can't see too far
            if abs(peds(p).x - cross_pos) < cross_width/2 + 12
                % check if ped in danger zone
                if abs(peds(p).y) < 12
                    car_time = (cross_pos - car.x) / max(car.v, 0.1);
                    ped_future_y = peds(p).y + peds(p).vy * car_time;
                    
                    % realistic collision prediction
                    if abs(ped_future_y) < 8 && car_time < 6 && car_time > 0
                        danger = true;
                        break;
                    end
                    
                    % stop if ped very close to road
                    if abs(peds(p).y) < 6
                        danger = true;
                        break;
                    end
                end
            end
        end
    end
end

function leader_idx = findLeader(cars, car_idx, active_cars)
    leader_idx = [];
    min_dist = 1000;
    
    for i = 1:length(active_cars)
        n = active_cars(i);
        if n == car_idx
            continue;
        end
        
        if cars(n).x > cars(car_idx).x
            dist = cars(n).x - cars(car_idx).x;
            if dist < min_dist
                min_dist = dist;
                leader_idx = n;
            end
        end
    end
end

function drawSim(cars, peds, cross_pos, cross_width, controller, time)
    clf;
    hold on;
    
    % road
    fill([-80, 550, 550, -80], [-6, -6, 6, 6], [0.4 0.4 0.4]);
    
    % crossing
    fill([cross_pos-cross_width/2, cross_pos+cross_width/2, cross_pos+cross_width/2, cross_pos-cross_width/2], ...
         [-25, -25, 25, 25], [0.9 0.9 0.7], 'EdgeColor', 'k');
    
         % stop line
     stop_x = cross_pos - cross_width/2 - 15;
     plot([stop_x, stop_x], [-8, 8], 'r-', 'LineWidth', 2);
    
    % cars
    active_cars = find([cars.active]);
    stopped = 0;
    for n = active_cars
        if cars(n).stopped
            plot(cars(n).x, cars(n).y, 's', 'MarkerSize', 10, 'Color', 'r', 'MarkerFaceColor', 'r');
            stopped = stopped + 1;
        else
            plot(cars(n).x, cars(n).y, 's', 'MarkerSize', 8, 'Color', 'b', 'MarkerFaceColor', 'b');
        end
    end
    
    % peds
    if ~isempty(peds)
        active_peds = find([peds.active]);
        for p = active_peds
            plot(peds(p).x, peds(p).y, 'o', 'MarkerSize', 7, 'Color', 'g', 'MarkerFaceColor', 'g');
        end
    end
    
    xlim([-50, 450]);
    ylim([-30, 30]);
    title(sprintf('%s - t=%.1fs, stopped=%d', controller, time, stopped));
    xlabel('x (m)');
    ylabel('y (m)');
    grid on;
end

function plotResults(idm_data, mpc_data)
    figure('Position', [50, 50, 900, 700]);
    
    % positions
    subplot(2,2,1);
    car_names = fieldnames(idm_data.cars);
    for i = 1:min(4, length(car_names))
        h = idm_data.cars.(car_names{i});
        plot(h.t, h.x, 'r-'); hold on;
    end
    title('IDM Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    
    subplot(2,2,3);
    car_names = fieldnames(mpc_data.cars);
    for i = 1:min(4, length(car_names))
        h = mpc_data.cars.(car_names{i});
        plot(h.t, h.x, 'b-'); hold on;
    end
    title('MPC Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    
    % comparison
    subplot(2,2,2);
    bar([idm_data.total_fuel, mpc_data.total_fuel]);
    set(gca, 'XTickLabel', {'IDM', 'MPC'});
    title('Fuel Usage');
    ylabel('Fuel (L)');
    
    subplot(2,2,4);
    bar([idm_data.avg_speed, mpc_data.avg_speed]);
    set(gca, 'XTickLabel', {'IDM', 'MPC'});
    title('Avg Speed');
    ylabel('Speed (m/s)');
end

function showStats(idm_data, mpc_data)
    fprintf('\n--- Results ---\n');
    fprintf('IDM: fuel=%.2fL, speed=%.1fm/s, stops=%d\n', ...
        idm_data.total_fuel, idm_data.avg_speed, idm_data.stops);
    fprintf('MPC: fuel=%.2fL, speed=%.1fm/s, stops=%d\n', ...
        mpc_data.total_fuel, mpc_data.avg_speed, mpc_data.stops);
    
    fuel_diff = (idm_data.total_fuel - mpc_data.total_fuel) / idm_data.total_fuel * 100;
    speed_diff = (mpc_data.avg_speed - idm_data.avg_speed) / idm_data.avg_speed * 100;
    
         fprintf('\nMPC vs IDM:\n');
     if fuel_diff > 0
         fprintf('Fuel: %.1f%% better\n', fuel_diff);
     else
         fprintf('Fuel: %.1f%% worse\n', abs(fuel_diff));
     end
     
     if speed_diff > 0
         fprintf('Speed: %.1f%% faster\n', speed_diff);
     else
         fprintf('Speed: %.1f%% slower\n', abs(speed_diff));
     end
end 
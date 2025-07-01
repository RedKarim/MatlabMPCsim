close all
clear all

% setup
ncars = 20;
dt = 0.5;
sig_pos = 400;
end_pos = 1800;

% signal stuff  
cycle_time = 45;
green_time = 20;
yellow_time = 5;
red_time = 20;

% fuel constants
b0 = 0.156;  b1 = 2.450e-2;  b2 = -7.415e-4;  b3 = 5.975e-5;
c0 = 0.07224; c1 = 9.681e-2; c2 = 1.075e-3;

disp('Running IDM vs MPC comparison...')
fprintf('Cars: %d, Signal at: %d m\n', ncars, sig_pos);

% run IDM first
fprintf('\nRunning IDM...\n');
IDM_data = runSim('IDM', ncars, dt, sig_pos, end_pos, cycle_time, green_time, yellow_time);

% now MPC
fprintf('\nRunning MPC...\n');
MPC_data = runSim('MPC', ncars, dt, sig_pos, end_pos, cycle_time, green_time, yellow_time);

% plots
fprintf('\nMaking plots...\n');
makePlots(IDM_data, MPC_data, sig_pos);

% results
fprintf('\nResults:\n');
printStats(IDM_data, MPC_data);

function results = runSim(controller, ncars, dt, sig_pos, end_pos, cycle_time, green_time, yellow_time)
    % init cars
    cars = struct('X', {}, 'V', {}, 'A', {}, 'active', {}, 'id', {});
    for i = 1:ncars
        cars(i) = struct('X', -i*10, 'V', 10, 'A', 0, 'active', true, 'id', i);
    end
    
    car_data = {};
    green_times = [];
    car_history = struct();
    
    b0 = 0.156;  b1 = 2.450e-2;  b2 = -7.415e-4;  b3 = 5.975e-5;
    c0 = 0.07224; c1 = 9.681e-2; c2 = 1.075e-3;
    
    % plot setup
    f = figure('Name', sprintf('%s Sim', controller));
    set(f, 'position', [400, 400, 800, 400]);
    axis([0 end_pos 0 10]);
    hold on;
    
    road = plot([0 end_pos], [4.5 4.5], 'LineWidth', 30, 'color', [0.5, 0.5, 0.5]);
    signal = plot(sig_pos, 8, 'sr', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    text(sig_pos, 9, sprintf('Signal (%s)', controller), 'HorizontalAlignment', 'center');
    car_plot = plot([], [], 'sr', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.1, 1]);
    
    title(sprintf('%s Controller', controller));
    xlabel('Position (m)');
    
    time = 0;
    t = 0;
    last_state = "";
    Tmax = 240;
    for time=1:Tmax
        t = t + 1;
        time = t * dt;
        
        pause(0.01);
        
        % signal
        tc = mod(time, cycle_time);
        if tc < green_time
            state = "green";
        elseif tc < green_time + yellow_time
            state = "yellow";
        else
            state = "red";
        end
        
        % update signal
        colors = struct('red', 'r', 'yellow', 'y', 'green', 'g');
        try
            set(signal, 'MarkerFaceColor', colors.(state));
        catch
        end
        
        if ~strcmp(state, last_state) && strcmp(state, 'green')
            green_times(end+1) = time;
        end
        last_state = state;
        
        % cars
        active = find([cars.active]);
        if isempty(active)
            break;
        end
        
        % sort active cars by position (furthest first for proper following)
        active_positions = [cars(active).X];
        [~, sort_idx] = sort(active_positions, 'descend');
        active = active(sort_idx);
        
        for i = 1:length(active)
            n = active(i);
            
            % signal check
            stop_for_signal = false;
            if cars(n).X < sig_pos && (strcmp(state, 'red') || strcmp(state, 'yellow'))
                stop_for_signal = true;
            end
            
            if stop_for_signal
                if strcmp(controller, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, sig_pos, 0);
                    % avoid collision with car ahead
                    if i > 1
                        ahead = active(i-1);  % car ahead in sorted order
                        if cars(ahead).X > cars(n).X  % make sure it's actually ahead
                            cars(n).A = min(cars(n).A, IDM(cars(n).X, cars(n).V, cars(ahead).X, cars(ahead).V));
                        end
                    end
                else
                    [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, sig_pos, 0);
                    % avoid collision with car ahead
                    if i > 1
                        ahead = active(i-1);  % car ahead in sorted order
                        if cars(ahead).X > cars(n).X  % make sure it's actually ahead
                            [follow_acc, ~] = MPC(cars(n).X, cars(n).V, cars(ahead).X, cars(ahead).V);
                            cars(n).A = min(cars(n).A, follow_acc);
                        end
                    end
                end
            elseif i == 1  % leader (furthest car)
                if strcmp(controller, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, cars(n).X+1000, 20);
                else
                    [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(n).X+1000, 20);
                end
            else
                % follow car ahead
                ahead = active(i-1);
                if strcmp(controller, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, cars(ahead).X, cars(ahead).V);
                else
                    [cars(n).A, ~] = MPC(cars(n).X, cars(n).V, cars(ahead).X, cars(ahead).V);
                end
            end
        end
        
        % save data only for active cars within bounds
        for n = active
            if cars(n).X <= end_pos  % only save data if car hasn't reached end
                id = cars(n).id;
                if ~isfield(car_history, sprintf('car%d', id))
                    car_history.(sprintf('car%d', id)) = struct('time', [], 'v', [], 'a', [], 'fuel', []);
                end
                
                v = cars(n).V;
                a = cars(n).A;
                ubar = max(a, 0);
                fc = b0 + b1*v + b2*v^2 + b3*v^3 + ubar * (c0 + c1*v + c2*v^2);
                
                car_history.(sprintf('car%d', id)).time(end+1) = time;
                car_history.(sprintf('car%d', id)).v(end+1) = v;
                car_history.(sprintf('car%d', id)).a(end+1) = a;
                car_history.(sprintf('car%d', id)).fuel(end+1) = fc;
            end
        end
        
        % move cars
        x_plot = [];
        v_plot = [];
        a_plot = [];
        for n = active
            cars(n).X = cars(n).X + cars(n).V * dt + 0.5 * cars(n).A * dt^2;
            cars(n).V = cars(n).V + cars(n).A * dt;
            cars(n).V = max(cars(n).V, 0);
            cars(n).A = min(max(cars(n).A, -8), 2);
            
            if cars(n).X > end_pos
                cars(n).active = false;
                continue;
            end
            
            if cars(n).active && cars(n).X >= 0 && cars(n).X <= end_pos
                x_plot(end+1) = cars(n).X;
                v_plot(end+1) = cars(n).V;
                a_plot(end+1) = cars(n).A;
            end
        end
        
        % update viz
        try
            delete(car_plot);
            if ~isempty(x_plot)
                car_plot = plot(x_plot, ones(size(x_plot))*5, 'sr', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.1, 1]);
            else
                car_plot = plot([], [], 'sr', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.1, 1]);
            end
            title(sprintf('%s - Time: %.1fs, Signal: %s', controller, time, upper(string(state))));
        catch
        end
        
        car_data{end+1} = [time, x_plot, v_plot, a_plot];
    end
    
    fprintf('Done with %s\n', controller);
    pause(1);
    try
        close(f);
    catch
    end
    
    results = processData(car_data, car_history, green_times, controller, dt);
end

function results = processData(car_data, car_history, green_times, controller, dt)
    % make data matrices for plotting
    num_steps = length(car_data);
    if num_steps == 0
        results = struct();
        return;
    end
    
    max_cars = max(cellfun(@(c) (size(c,2)-1)/3, car_data));
    pos_data = NaN(num_steps, max_cars);
    vel_data = NaN(num_steps, max_cars);
    acc_data = NaN(num_steps, max_cars);
    time_vec = zeros(num_steps,1);
    
    for i = 1:num_steps
        entry = car_data{i};
        time_vec(i) = entry(1);
        ncars = (size(entry,2)-1)/3;
        if ncars > 0
            pos_data(i,1:ncars) = entry(2:1+ncars);
            vel_data(i,1:ncars) = entry(2+ncars:1+2*ncars);
            acc_data(i,1:ncars) = entry(2+2*ncars:end);
        end
    end
    
    % calc fuel stats
    car_ids = fieldnames(car_history);
    num_cars = length(car_ids);
    fuel_per_car = zeros(1, num_cars);
    vel_per_car = zeros(1, num_cars);
    idle_per_car = zeros(1, num_cars);
    
    for i = 1:num_cars
        hist = car_history.(car_ids{i});
        fuel_per_car(i) = sum(hist.fuel) * dt;
        vel_per_car(i) = mean(hist.v);
        idle_per_car(i) = sum(hist.v < 1.5) * dt;
    end
    
    % pack results
    results = struct();
    results.controller = controller;
    results.TimeVec = time_vec;
    results.PosData = pos_data;
    results.VelData = vel_data;
    results.AccData = acc_data;
    results.GreenTimes = green_times;
    results.CarHistory = car_history;
    results.TotalFuelPerCar = fuel_per_car;
    results.AvgVelPerCar = vel_per_car;
    results.IdleTimePerCar = idle_per_car;
    results.TotalFuel = sum(fuel_per_car);
    results.AvgVel = mean(vel_per_car);
    results.AvgIdleTime = mean(idle_per_car);
end

function makePlots(IDM_data, MPC_data, sig_pos)
    
    % positions
    figure('Position', [100, 100, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_data.TimeVec, IDM_data.PosData, 'b-', 'LineWidth', 1);
    title('IDM: Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    hold on;
    for k = 1:length(IDM_data.GreenTimes)
        plot([IDM_data.GreenTimes(k), IDM_data.GreenTimes(k)], [0, 1000], '--g', 'LineWidth', 1);
    end
    plot(xlim, [sig_pos, sig_pos], ':k', 'LineWidth', 2);
    ylim([0 1000]);
    grid on;
    
    subplot(1,2,2);
    plot(MPC_data.TimeVec, MPC_data.PosData, 'r-', 'LineWidth', 1);
    title('MPC: Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    hold on;
    for k = 1:length(MPC_data.GreenTimes)
        plot([MPC_data.GreenTimes(k), MPC_data.GreenTimes(k)], [0, 1000], '--g', 'LineWidth', 1);
    end
    plot(xlim, [sig_pos, sig_pos], ':k', 'LineWidth', 2);
    ylim([0 1000]);
    grid on;
    
    % velocities
    figure('Position', [100, 200, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_data.TimeVec, IDM_data.VelData, 'b-', 'LineWidth', 1);
    title('IDM: Velocities');
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    grid on;
    
    subplot(1,2,2);
    plot(MPC_data.TimeVec, MPC_data.VelData, 'r-', 'LineWidth', 1);
    title('MPC: Velocities');
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    grid on;
    
    % accelerations
    figure('Position', [100, 300, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_data.TimeVec, IDM_data.AccData, 'b-', 'LineWidth', 1);
    title('IDM: Accelerations');
    xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
    grid on;
    
    subplot(1,2,2);
    plot(MPC_data.TimeVec, MPC_data.AccData, 'r-', 'LineWidth', 1);
    title('MPC: Accelerations');
    xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
    grid on;
    
    % fuel plots
    figure('Position', [100, 400, 1200, 600]);
    
    subplot(2,2,1);
    plot(1:length(IDM_data.TotalFuelPerCar), IDM_data.TotalFuelPerCar, 'bo-', 'MarkerSize', 6);
    hold on;
    plot(1:length(MPC_data.TotalFuelPerCar), MPC_data.TotalFuelPerCar, 'ro-', 'MarkerSize', 6);
    title('Total Fuel Per Car');
    xlabel('Car Number'); ylabel('Fuel (mL)');
    legend('IDM', 'MPC', 'Location', 'best');
    grid on;
    
    subplot(2,2,2);
    plot(1:length(IDM_data.AvgVelPerCar), IDM_data.AvgVelPerCar, 'bo-', 'MarkerSize', 6);
    hold on;
    plot(1:length(MPC_data.AvgVelPerCar), MPC_data.AvgVelPerCar, 'ro-', 'MarkerSize', 6);
    title('Avg Velocity Per Car');
    xlabel('Car Number'); ylabel('Velocity (m/s)');
    legend('IDM', 'MPC', 'Location', 'best');
    grid on;
    
    subplot(2,2,3);
    plot(1:length(IDM_data.IdleTimePerCar), IDM_data.IdleTimePerCar, 'bo-', 'MarkerSize', 6);
    hold on;
    plot(1:length(MPC_data.IdleTimePerCar), MPC_data.IdleTimePerCar, 'ro-', 'MarkerSize', 6);
    title('Idle Time Per Car');
    xlabel('Car Number'); ylabel('Idle Time (s)');
    legend('IDM', 'MPC', 'Location', 'best');
    grid on;
    
    % fuel over time
    figure('Position', [100, 500, 1200, 400]);
    
    subplot(1,2,1);
    car_ids = fieldnames(IDM_data.CarHistory);
    hold on;
    for i = 1:length(car_ids)
        plot(IDM_data.CarHistory.(car_ids{i}).time, IDM_data.CarHistory.(car_ids{i}).fuel, 'b-');
    end
    title('IDM: Fuel Over Time');
    xlabel('Time (s)'); ylabel('Fuel (mL/s)');
    grid on;
    
    subplot(1,2,2);
    car_ids = fieldnames(MPC_data.CarHistory);
    hold on;
    for i = 1:length(car_ids)
        plot(MPC_data.CarHistory.(car_ids{i}).time, MPC_data.CarHistory.(car_ids{i}).fuel, 'r-');
    end
    title('MPC: Fuel Over Time');
    xlabel('Time (s)'); ylabel('Fuel (mL/s)');
    grid on;
end

function printStats(IDM_data, MPC_data)
    fprintf('\n--- Results ---\n');
    fprintf('IDM:\n');
    fprintf('  Fuel: %.2f mL\n', IDM_data.TotalFuel);
    fprintf('  Avg speed: %.2f m/s\n', IDM_data.AvgVel);
    fprintf('  Idle time: %.2f s\n', IDM_data.AvgIdleTime);
    fprintf('  Duration: %.2f s\n', IDM_data.TimeVec(end));
    
    fprintf('\nMPC:\n');
    fprintf('  Fuel: %.2f mL\n', MPC_data.TotalFuel);
    fprintf('  Avg speed: %.2f m/s\n', MPC_data.AvgVel);
    fprintf('  Idle time: %.2f s\n', MPC_data.AvgIdleTime);
    fprintf('  Duration: %.2f s\n', MPC_data.TimeVec(end));
    
    fprintf('\nComparison:\n');
    fuel_diff = ((IDM_data.TotalFuel - MPC_data.TotalFuel) / IDM_data.TotalFuel) * 100;
    vel_diff = ((MPC_data.AvgVel - IDM_data.AvgVel) / IDM_data.AvgVel) * 100;
    idle_diff = ((IDM_data.AvgIdleTime - MPC_data.AvgIdleTime) / IDM_data.AvgIdleTime) * 100;
    
    if fuel_diff > 0
        fprintf('  Fuel: %.1f%% better\n', fuel_diff);
    else
        fprintf('  Fuel: %.1f%% worse\n', abs(fuel_diff));
    end
    
    if vel_diff > 0
        fprintf('  Speed: %.1f%% faster\n', vel_diff);
    else
        fprintf('  Speed: %.1f%% slower\n', abs(vel_diff));
    end
    
    if idle_diff > 0
        fprintf('  Idle: %.1f%% less\n', idle_diff);
    else
        fprintf('  Idle: %.1f%% more\n', abs(idle_diff));
    end
end

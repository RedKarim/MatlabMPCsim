close all
clear all

% Basic setup
ncars = 20;
dt = 0.5;
sig_pos = 400;
end_pos = 800;

% Signal timing
cycle_time = 45;
green_time = 20;
yellow_time = 5;
red_time = 20;

% fuel constants (from some paper)
b0 = 0.156;  b1 = 2.450e-2;  b2 = -7.415e-4;  b3 = 5.975e-5;
c0 = 0.07224; c1 = 9.681e-2; c2 = 1.075e-3;

disp('Running IDM vs MPC comparison...')
fprintf('Cars: %d, Signal at: %d m\n', ncars, sig_pos);

% Run IDM first
fprintf('\nRunning IDM simulation...\n');
IDM_results = runSim('IDM', ncars, dt, sig_pos, end_pos, cycle_time, green_time, yellow_time);

% Now MPC
fprintf('\nRunning MPC simulation...\n');
MPC_results = runSim('MPC', ncars, dt, sig_pos, end_pos, cycle_time, green_time, yellow_time);

% Make plots
fprintf('\nGenerating plots...\n');
makePlots(IDM_results, MPC_results, sig_pos);

% Print results
fprintf('\nResults:\n');
printStats(IDM_results, MPC_results);

function results = runSim(controller, ncars, dt, sig_pos, end_pos, cycle_time, green_time, yellow_time)
    % setup cars
    cars = struct('X', {}, 'V', {}, 'A', {}, 'active', {}, 'id', {});
    for i = 1:ncars
        cars(i) = struct('X', -i*10, 'V', 10, 'A', 0, 'active', true, 'id', i);
    end
    
    CarData = {};
    GreenTimes = [];
    CarHistory = struct();
    
    b0 = 0.156;  b1 = 2.450e-2;  b2 = -7.415e-4;  b3 = 5.975e-5;
    c0 = 0.07224; c1 = 9.681e-2; c2 = 1.075e-3;
    
    % visualization stuff
    f = figure('Name', sprintf('%s Sim', controller));
    set(f, 'position', [400, 400, 800, 400]);
    axis([0 end_pos 0 10]);
    hold on;
    
    p_road = plot([0 end_pos], [4.5 4.5], 'LineWidth', 30, 'color', [0.5, 0.5, 0.5]);
    p_signal = plot(sig_pos, 8, 'sr', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    text(sig_pos, 9, sprintf('Signal (%s)', controller), 'HorizontalAlignment', 'center');
    p_cars = plot([], [], 'sr', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.1, 1]);
    
    title(sprintf('%s Controller', controller));
    xlabel('Position (m)');
    
    time = 0;
    t = 0;
    last_state = "";
    
    while true
        t = t + 1;
        time = t * dt;
        pause(0.01);
        
        % signal logic
        tc = mod(time, cycle_time);
        if tc < green_time
            state = "green";
        elseif tc < green_time + yellow_time
            state = "yellow";
        else
            state = "red";
        end
        
        % update signal color
        colors = struct('red', 'r', 'yellow', 'y', 'green', 'g');
        try
            set(p_signal, 'MarkerFaceColor', colors.(state));
        catch
        end
        
        if ~strcmp(state, last_state) && strcmp(state, 'green')
            GreenTimes(end+1) = time;
        end
        last_state = state;
        
        % car logic
        active = find([cars.active]);
        if isempty(active)
            break;
        end
        
        for n = active
            % check signal
            stop_signal = false;
            if cars(n).X < sig_pos && (strcmp(state, 'red') || strcmp(state, 'yellow'))
                stop_signal = true;
            end
            
            if stop_signal
                if strcmp(controller, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, sig_pos, 0);
                else
                    cars(n).A = MPC(cars(n).X, cars(n).V, sig_pos, 0);
                end
            elseif n == active(1)
                % lead car
                if strcmp(controller, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, cars(n).X+1000, 20);
                else
                    cars(n).A = MPC(cars(n).X, cars(n).V, cars(n).X+1000, 20);
                end
            else
                % follow car ahead
                prev = active(find(active == n) - 1);
                if strcmp(controller, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, cars(prev).X, cars(prev).V);
                else
                    cars(n).A = MPC(cars(n).X, cars(n).V, cars(prev).X, cars(prev).V);
                end
            end
            
            % track data
            id = cars(n).id;
            if ~isfield(CarHistory, sprintf('car%d', id))
                CarHistory.(sprintf('car%d', id)) = struct('time', [], 'v', [], 'a', [], 'fuel', []);
            end
            
            v = cars(n).V;
            a = cars(n).A;
            ubar = max(a, 0);
            fc = b0 + b1*v + b2*v^2 + b3*v^3 + ubar * (c0 + c1*v + c2*v^2);
            
            CarHistory.(sprintf('car%d', id)).time(end+1) = time;
            CarHistory.(sprintf('car%d', id)).v(end+1) = v;
            CarHistory.(sprintf('car%d', id)).a(end+1) = a;
            CarHistory.(sprintf('car%d', id)).fuel(end+1) = fc;
        end
        
        % update cars
        X_plot = [];
        V_plot = [];
        A_plot = [];
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
                X_plot(end+1) = cars(n).X;
                V_plot(end+1) = cars(n).V;
                A_plot(end+1) = cars(n).A;
            end
        end
        
        % update plot
        try
            delete(p_cars);
            if ~isempty(X_plot)
                p_cars = plot(X_plot, ones(size(X_plot))*5, 'sr', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.1, 1]);
            else
                p_cars = plot([], [], 'sr', 'MarkerSize', 10, 'MarkerFaceColor', [0.5, 0.1, 1]);
            end
            title(sprintf('%s - Time: %.1fs, Signal: %s', controller, time, upper(string(state))));
        catch
        end
        
        CarData{end+1} = [time, X_plot, V_plot, A_plot];
    end
    
    fprintf('Done with %s\n', controller);
    pause(1);
    try
        close(f);
    catch
    end
    
    results = processData(CarData, CarHistory, GreenTimes, controller, dt);
end

function results = processData(CarData, CarHistory, GreenTimes, controller, dt)
    % Align variable-length CarData into matrices for plotting
    numSteps = length(CarData);
    if numSteps == 0
        results = struct();
        return;
    end
    
    maxCars = max(cellfun(@(c) (size(c,2)-1)/3, CarData));
    PosData = NaN(numSteps, maxCars);
    VelData = NaN(numSteps, maxCars);
    AccData = NaN(numSteps, maxCars);
    TimeVec = zeros(numSteps,1);
    
    for i = 1:numSteps
        entry = CarData{i};
        TimeVec(i) = entry(1);
        nCars = (size(entry,2)-1)/3;
        if nCars > 0
            PosData(i,1:nCars) = entry(2:1+nCars);
            VelData(i,1:nCars) = entry(2+nCars:1+2*nCars);
            AccData(i,1:nCars) = entry(2+2*nCars:end);
        end
    end
    
    % Calculate fuel consumption statistics
    car_ids = fieldnames(CarHistory);
    num_cars = length(car_ids);
    TotalFuelPerCar = zeros(1, num_cars);
    AvgVelPerCar = zeros(1, num_cars);
    IdleTimePerCar = zeros(1, num_cars);
    
    for i = 1:num_cars
        car_hist = CarHistory.(car_ids{i});
        TotalFuelPerCar(i) = sum(car_hist.fuel) * dt;
        AvgVelPerCar(i) = mean(car_hist.v);
        IdleTimePerCar(i) = sum(car_hist.v < 0.1) * dt;
    end
    
    % Store results
    results = struct();
    results.controller = controller;
    results.TimeVec = TimeVec;
    results.PosData = PosData;
    results.VelData = VelData;
    results.AccData = AccData;
    results.GreenTimes = GreenTimes;
    results.CarHistory = CarHistory;
    results.TotalFuelPerCar = TotalFuelPerCar;
    results.AvgVelPerCar = AvgVelPerCar;
    results.IdleTimePerCar = IdleTimePerCar;
    results.TotalFuel = sum(TotalFuelPerCar);
    results.AvgVel = mean(AvgVelPerCar);
    results.AvgIdleTime = mean(IdleTimePerCar);
end

function makePlots(IDM_results, MPC_results, sig_pos)
    
    % positions
    figure('Position', [100, 100, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_results.TimeVec, IDM_results.PosData, 'b-', 'LineWidth', 1);
    title('IDM: Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    hold on;
    for k = 1:length(IDM_results.GreenTimes)
        plot([IDM_results.GreenTimes(k), IDM_results.GreenTimes(k)], [0, 1000], '--g', 'LineWidth', 1);
    end
    plot(xlim, [sig_pos, sig_pos], ':k', 'LineWidth', 2);
    ylim([0 1000]);
    grid on;
    
    subplot(1,2,2);
    plot(MPC_results.TimeVec, MPC_results.PosData, 'r-', 'LineWidth', 1);
    title('MPC: Positions');
    xlabel('Time (s)'); ylabel('Position (m)');
    hold on;
    for k = 1:length(MPC_results.GreenTimes)
        plot([MPC_results.GreenTimes(k), MPC_results.GreenTimes(k)], [0, 1000], '--g', 'LineWidth', 1);
    end
    plot(xlim, [sig_pos, sig_pos], ':k', 'LineWidth', 2);
    ylim([0 1000]);
    grid on;
    
    % velocities
    figure('Position', [100, 200, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_results.TimeVec, IDM_results.VelData, 'b-', 'LineWidth', 1);
    title('IDM: Velocities');
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    grid on;
    
    subplot(1,2,2);
    plot(MPC_results.TimeVec, MPC_results.VelData, 'r-', 'LineWidth', 1);
    title('MPC: Velocities');
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    grid on;
    
    % accelerations
    figure('Position', [100, 300, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_results.TimeVec, IDM_results.AccData, 'b-', 'LineWidth', 1);
    title('IDM: Accelerations');
    xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
    grid on;
    
    subplot(1,2,2);
    plot(MPC_results.TimeVec, MPC_results.AccData, 'r-', 'LineWidth', 1);
    title('MPC: Accelerations');
    xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
    grid on;
    
    %% Plot 4: Fuel Consumption Comparison
    figure('Position', [100, 400, 1200, 600]);
    
    subplot(2,2,1);
    plot(1:length(IDM_results.TotalFuelPerCar), IDM_results.TotalFuelPerCar, 'bo-', 'MarkerSize', 6);
    hold on;
    plot(1:length(MPC_results.TotalFuelPerCar), MPC_results.TotalFuelPerCar, 'ro-', 'MarkerSize', 6);
    title('Total Fuel Consumed Per Car');
    xlabel('Car Number'); ylabel('Total Fuel (mL)');
    legend('IDM', 'MPC', 'Location', 'best');
    grid on;
    
    subplot(2,2,2);
    plot(1:length(IDM_results.AvgVelPerCar), IDM_results.AvgVelPerCar, 'bo-', 'MarkerSize', 6);
    hold on;
    plot(1:length(MPC_results.AvgVelPerCar), MPC_results.AvgVelPerCar, 'ro-', 'MarkerSize', 6);
    title('Average Velocity Per Car');
    xlabel('Car Number'); ylabel('Average Velocity (m/s)');
    legend('IDM', 'MPC', 'Location', 'best');
    grid on;
    
    subplot(2,2,3);
    plot(1:length(IDM_results.IdleTimePerCar), IDM_results.IdleTimePerCar, 'bo-', 'MarkerSize', 6);
    hold on;
    plot(1:length(MPC_results.IdleTimePerCar), MPC_results.IdleTimePerCar, 'ro-', 'MarkerSize', 6);
    title('Idle Time Per Car');
    xlabel('Car Number'); ylabel('Idle Time (s)');
    legend('IDM', 'MPC', 'Location', 'best');
    grid on;
    
    %% Plot 5: Fuel Consumption Over Time by Controller
    figure('Position', [100, 500, 1200, 400]);
    
    subplot(1,2,1);
    car_ids = fieldnames(IDM_results.CarHistory);
    hold on;
    for i = 1:length(car_ids)
        plot(IDM_results.CarHistory.(car_ids{i}).time, IDM_results.CarHistory.(car_ids{i}).fuel, 'b-');
    end
    title('IDM: Fuel Consumption Over Time');
    xlabel('Time (s)'); ylabel('Fuel Consumption (mL/s)');
    grid on;
    
    subplot(1,2,2);
    car_ids = fieldnames(MPC_results.CarHistory);
    hold on;
    for i = 1:length(car_ids)
        plot(MPC_results.CarHistory.(car_ids{i}).time, MPC_results.CarHistory.(car_ids{i}).fuel, 'r-');
    end
    title('MPC: Fuel Consumption Over Time');
    xlabel('Time (s)'); ylabel('Fuel Consumption (mL/s)');
    grid on;
end

function printStats(IDM_results, MPC_results)
    fprintf('\n--- Results ---\n');
    fprintf('IDM:\n');
    fprintf('  Fuel: %.2f mL\n', IDM_results.TotalFuel);
    fprintf('  Avg speed: %.2f m/s\n', IDM_results.AvgVel);
    fprintf('  Idle time: %.2f s\n', IDM_results.AvgIdleTime);
    fprintf('  Duration: %.2f s\n', IDM_results.TimeVec(end));
    
    fprintf('\nMPC:\n');
    fprintf('  Fuel: %.2f mL\n', MPC_results.TotalFuel);
    fprintf('  Avg speed: %.2f m/s\n', MPC_results.AvgVel);
    fprintf('  Idle time: %.2f s\n', MPC_results.AvgIdleTime);
    fprintf('  Duration: %.2f s\n', MPC_results.TimeVec(end));
    
    fprintf('\nMPC vs IDM:\n');
    fuel_diff = ((IDM_results.TotalFuel - MPC_results.TotalFuel) / IDM_results.TotalFuel) * 100;
    vel_diff = ((MPC_results.AvgVel - IDM_results.AvgVel) / IDM_results.AvgVel) * 100;
    idle_diff = ((IDM_results.AvgIdleTime - MPC_results.AvgIdleTime) / IDM_results.AvgIdleTime) * 100;
    
    if fuel_diff > 0
        fprintf('  Fuel: %.1f%% better\n', fuel_diff);
    else
        fprintf('  Fuel: %.1f%% worse\n', abs(fuel_diff));
    end
    
    if vel_diff > 0
        fprintf('  Speed: %.1f%% better\n', vel_diff);
    else
        fprintf('  Speed: %.1f%% worse\n', abs(vel_diff));
    end
    
    if idle_diff > 0
        fprintf('  Idle: %.1f%% better\n', idle_diff);
    else
        fprintf('  Idle: %.1f%% worse\n', abs(idle_diff));
    end
end

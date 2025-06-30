close all
clear all

%% Simulation Parameters
num_cars = 30;           % Fixed number of cars
dt = 0.5;                % Time step
signal_position = 400;   % Single signal position
simulation_end_pos = 800; % End simulation when cars reach this position

% Traffic signal parameters
cycle_length = 45;       % Total cycle duration in seconds
green_duration = 20;     % Green duration in seconds
yellow_duration = 5;     % Yellow duration in seconds
red_duration = 20;       % Red duration in seconds

% Fuel consumption constants
b0 = 0.156;  b1 = 2.450e-2;  b2 = -7.415e-4;  b3 = 5.975e-5;
c0 = 0.07224; c1 = 9.681e-2; c2 = 1.075e-3;

fprintf('Running IDM vs MPC Comparison Simulation...\n');
fprintf('Number of cars: %d\n', num_cars);
fprintf('Signal position: %d m\n', signal_position);

%% Run Simulation with IDM
fprintf('\nRunning simulation with IDM controller...\n');
[IDM_results] = runSimulation('IDM', num_cars, dt, signal_position, simulation_end_pos, cycle_length, green_duration, yellow_duration);

%% Run Simulation with MPC
fprintf('Running simulation with MPC controller...\n');
[MPC_results] = runSimulation('MPC', num_cars, dt, signal_position, simulation_end_pos, cycle_length, green_duration, yellow_duration);

%% Create Comparison Plots
createComparisonPlots(IDM_results, MPC_results, signal_position);

%% Print Summary Statistics
printSummaryStats(IDM_results, MPC_results);

%% Simulation Function
function [results] = runSimulation(controller_type, num_cars, dt, signal_position, simulation_end_pos, cycle_length, green_duration, yellow_duration)
    % Initialize cars - spawn all 30 cars at start with 10m spacing
    cars = struct('X', {}, 'V', {}, 'A', {}, 'active', {}, 'id', {});
    for i = 1:num_cars
        cars(i) = struct('X', -i*10, 'V', 10, 'A', 0, 'active', true, 'id', i);
    end
    
    % Initialize tracking
    CarData = {};
    GreenTimes = [];
    CarHistory = struct();
    
    % Fuel consumption constants
    b0 = 0.156;  b1 = 2.450e-2;  b2 = -7.415e-4;  b3 = 5.975e-5;
    c0 = 0.07224; c1 = 9.681e-2; c2 = 1.075e-3;
    
    current_time = 0;
    t = 0;
    last_signal_state = "";
    
    % Main simulation loop
    while true
        t = t + 1;
        current_time = t * dt;
        
        % Traffic signal logic
        time_in_cycle = mod(current_time, cycle_length);
        if time_in_cycle < green_duration
            signal_state = "green";
        elseif time_in_cycle < green_duration + yellow_duration
            signal_state = "yellow";
        else
            signal_state = "red";
        end
        
        % Record green transitions
        if ~strcmp(signal_state, last_signal_state) && strcmp(signal_state, 'green')
            GreenTimes(end+1) = current_time;
        end
        last_signal_state = signal_state;
        
        % Car control logic
        active_cars = find([cars.active]);
        if isempty(active_cars)
            break; % All cars have finished
        end
        
        for n = active_cars
            % Check if car needs to stop for signal
            stop_for_signal = false;
            if cars(n).X < signal_position && (strcmp(signal_state, 'red') || strcmp(signal_state, 'yellow'))
                stop_for_signal = true;
            end
            
            if stop_for_signal
                % Stop for signal
                if strcmp(controller_type, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, signal_position, 0);
                else % MPC
                    cars(n).A = MPC(cars(n).X, cars(n).V, signal_position, 0);
                end
            elseif n == active_cars(1)
                % Lead car, no car in front
                if strcmp(controller_type, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, cars(n).X+1000, 20);
                else % MPC
                    cars(n).A = MPC(cars(n).X, cars(n).V, cars(n).X+1000, 20);
                end
            else
                % Follow the car in front
                prev_car_idx = active_cars(find(active_cars == n) - 1);
                if strcmp(controller_type, 'IDM')
                    cars(n).A = IDM(cars(n).X, cars(n).V, cars(prev_car_idx).X, cars(prev_car_idx).V);
                else % MPC
                    cars(n).A = MPC(cars(n).X, cars(n).V, cars(prev_car_idx).X, cars(prev_car_idx).V);
                end
            end
            
            % Track car history
            car_id = cars(n).id;
            if ~isfield(CarHistory, sprintf('car%d', car_id))
                CarHistory.(sprintf('car%d', car_id)) = struct('time', [], 'v', [], 'a', [], 'fuel', []);
            end
            
            % Calculate fuel consumption
            v = cars(n).V;
            a = cars(n).A;
            u_bar = max(a, 0);
            fc = b0 + b1*v + b2*v^2 + b3*v^3 + u_bar * (c0 + c1*v + c2*v^2);
            
            CarHistory.(sprintf('car%d', car_id)).time(end+1) = current_time;
            CarHistory.(sprintf('car%d', car_id)).v(end+1) = v;
            CarHistory.(sprintf('car%d', car_id)).a(end+1) = a;
            CarHistory.(sprintf('car%d', car_id)).fuel(end+1) = fc;
        end
        
        % Update positions and velocities
        X_plot = [];
        V_plot = [];
        A_plot = [];
        for n = active_cars
            % Update dynamics
            cars(n).X = cars(n).X + cars(n).V * dt + 0.5 * cars(n).A * dt^2;
            cars(n).V = cars(n).V + cars(n).A * dt;
            cars(n).V = max(cars(n).V, 0); % No negative speeds
            cars(n).A = min(max(cars(n).A, -8), 2); % Bound acceleration
            
            % Check if car has exited simulation
            if cars(n).X > simulation_end_pos
                cars(n).active = false;
                continue;
            end
            
            % Collect data for plotting
            if cars(n).active
                X_plot(end+1) = cars(n).X;
                V_plot(end+1) = cars(n).V;
                A_plot(end+1) = cars(n).A;
            end
        end
        
        % Store simulation data
        CarData{end+1} = [current_time, X_plot, V_plot, A_plot];
    end
    
    % Process results
    results = processResults(CarData, CarHistory, GreenTimes, controller_type, dt);
end

%% Process Results Function
function [results] = processResults(CarData, CarHistory, GreenTimes, controller_type, dt)
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
    results.controller = controller_type;
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

%% Create Comparison Plots Function
function createComparisonPlots(IDM_results, MPC_results, signal_position)
    
    %% Plot 1: Car Positions Over Time (Comparison)
    figure('Position', [100, 100, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_results.TimeVec, IDM_results.PosData, 'b-', 'LineWidth', 1);
    title('IDM: Car Positions Over Time');
    xlabel('Time (s)'); ylabel('Position (m)');
    hold on;
    % Signal green transitions
    for k = 1:length(IDM_results.GreenTimes)
        plot([IDM_results.GreenTimes(k), IDM_results.GreenTimes(k)], [0, 1000], '--g', 'LineWidth', 1);
    end
    % Signal position line
    plot(xlim, [signal_position, signal_position], ':k', 'LineWidth', 2);
    ylim([0 1000]);
    grid on;
    
    subplot(1,2,2);
    plot(MPC_results.TimeVec, MPC_results.PosData, 'r-', 'LineWidth', 1);
    title('MPC: Car Positions Over Time');
    xlabel('Time (s)'); ylabel('Position (m)');
    hold on;
    % Signal green transitions
    for k = 1:length(MPC_results.GreenTimes)
        plot([MPC_results.GreenTimes(k), MPC_results.GreenTimes(k)], [0, 1000], '--g', 'LineWidth', 1);
    end
    % Signal position line
    plot(xlim, [signal_position, signal_position], ':k', 'LineWidth', 2);
    ylim([0 1000]);
    grid on;
    
    %% Plot 2: Car Velocities Over Time (Comparison)
    figure('Position', [100, 200, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_results.TimeVec, IDM_results.VelData, 'b-', 'LineWidth', 1);
    title('IDM: Car Velocities Over Time');
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    grid on;
    
    subplot(1,2,2);
    plot(MPC_results.TimeVec, MPC_results.VelData, 'r-', 'LineWidth', 1);
    title('MPC: Car Velocities Over Time');
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    grid on;
    
    %% Plot 3: Car Accelerations Over Time (Comparison)
    figure('Position', [100, 300, 1200, 400]);
    
    subplot(1,2,1);
    plot(IDM_results.TimeVec, IDM_results.AccData, 'b-', 'LineWidth', 1);
    title('IDM: Car Accelerations Over Time');
    xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
    grid on;
    
    subplot(1,2,2);
    plot(MPC_results.TimeVec, MPC_results.AccData, 'r-', 'LineWidth', 1);
    title('MPC: Car Accelerations Over Time');
    xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
    grid on;
    
    %% Plot 4: Fuel Consumption Comparison
    figure('Position', [100, 400, 1200, 800]);
    
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
    
    subplot(2,2,4);
    categories = {'Total Fuel (mL)', 'Avg Velocity (m/s)', 'Avg Idle Time (s)'};
    IDM_values = [IDM_results.TotalFuel, IDM_results.AvgVel, IDM_results.AvgIdleTime];
    MPC_values = [MPC_results.TotalFuel, MPC_results.AvgVel, MPC_results.AvgIdleTime];
    
    x = 1:length(categories);
    width = 0.35;
    bar(x - width/2, IDM_values, width, 'FaceColor', 'b', 'DisplayName', 'IDM');
    hold on;
    bar(x + width/2, MPC_values, width, 'FaceColor', 'r', 'DisplayName', 'MPC');
    
    title('Overall Performance Comparison');
    xlabel('Metrics');
    ylabel('Values');
    xticks(x);
    xticklabels(categories);
    xtickangle(45);
    legend();
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

%% Print Summary Statistics Function
function printSummaryStats(IDM_results, MPC_results)
    fprintf('\n=== SIMULATION RESULTS SUMMARY ===\n');
    fprintf('IDM Controller:\n');
    fprintf('  Total Fuel Consumption: %.2f mL\n', IDM_results.TotalFuel);
    fprintf('  Average Velocity: %.2f m/s\n', IDM_results.AvgVel);
    fprintf('  Average Idle Time: %.2f s\n', IDM_results.AvgIdleTime);
    fprintf('  Simulation Duration: %.2f s\n', IDM_results.TimeVec(end));
    
    fprintf('\nMPC Controller:\n');
    fprintf('  Total Fuel Consumption: %.2f mL\n', MPC_results.TotalFuel);
    fprintf('  Average Velocity: %.2f m/s\n', MPC_results.AvgVel);
    fprintf('  Average Idle Time: %.2f s\n', MPC_results.AvgIdleTime);
    fprintf('  Simulation Duration: %.2f s\n', MPC_results.TimeVec(end));
    
    fprintf('\nComparison (MPC vs IDM):\n');
    fuel_improvement = ((IDM_results.TotalFuel - MPC_results.TotalFuel) / IDM_results.TotalFuel) * 100;
    vel_improvement = ((MPC_results.AvgVel - IDM_results.AvgVel) / IDM_results.AvgVel) * 100;
    idle_improvement = ((IDM_results.AvgIdleTime - MPC_results.AvgIdleTime) / IDM_results.AvgIdleTime) * 100;
    
    if fuel_improvement > 0
        fprintf('  Fuel Consumption: %.2f%% better\n', fuel_improvement);
    else
        fprintf('  Fuel Consumption: %.2f%% worse\n', abs(fuel_improvement));
    end
    
    if vel_improvement > 0
        fprintf('  Average Velocity: %.2f%% better\n', vel_improvement);
    else
        fprintf('  Average Velocity: %.2f%% worse\n', abs(vel_improvement));
    end
    
    if idle_improvement > 0
        fprintf('  Idle Time: %.2f%% better\n', idle_improvement);
    else
        fprintf('  Idle Time: %.2f%% worse\n', abs(idle_improvement));
    end
    fprintf('=====================================\n');
end

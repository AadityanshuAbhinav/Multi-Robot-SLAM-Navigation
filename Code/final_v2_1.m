clear; clc; close all;

%% Scenario Selector
scenario = 'corridor_maze'; % Options: 'zigzag', 'two_circles', 'semicircle','corridor_maze'

%% --- Simulation and Control Parameters ---

params.N = 10;             % Number of robots (agents) in the swarm
params.dt = 0.02;          % Simulation timestep in seconds
params.t_max = 150;        % Total simulation duration (seconds)
% --- Control Gains ---
params.c1_alpha = 0.5;               % Gain for inter-agent spacing (gradient term)
params.c2_alpha = 1 * sqrt(10);     % Gain for inter-agent velocity consensus (damping term)
params.c1_beta = 15;                % Gain for obstacle repulsion (stronger keeps robots away)
params.c2_beta = 20 * sqrt(50);      % Damping gain for obstacle influence (smooths avoidance)
params.c1_gamma = 2.30;              % Gain for goal attraction (pull toward virtual/real goal)
params.c2_gamma = 0.1;              % Damping for goal motion (reduces overshoot near goal)

% --- Physical & Sensing Parameters ---
params.rs = 20;             % Sensing (obstacle detection) radius in meters
params.rc = 30;             % Communication radius — how far robots can detect neighbors
params.v_max = 4;           % Maximum linear velocity (m/s)
params.d_alpha = 10;         % Desired inter-agent spacing distance (m)
params.epsilon = 0.1;       % Sigma-norm smoothness parameter (avoids singularities)
params.h_alpha = 0.2;       % Bump function cutoff for interaction strength (0–1 range)
params.delta_turn = deg2rad(20);  % Angular step for rotation during wall-following (radians)

% --- Stuck / Escape Behavior Parameters ---
params.stuck_check_interval = 20.0;      % How often (s) each robot checks if it's stuck
params.stuck_progress_eps = 0.04;       % Minimum progress (m) per interval to not be considered stuck
params.stuck_time_threshold = 2.0;      % Time (s) a robot must show no progress to be declared stuck
params.force_escape_group_frac = 0.6;   % Fraction of robots stuck before triggering a group escape maneuver
params.random_perturb = deg2rad(40);    % Random angular deviation (radians) added during escape to desynchronize motion
params.escape_rtan_scale = 1.6;         % Scaling factor for escape tangential radius (how far to orbit around obstacle)

% --- DSR (Delayed Self-Reinforcement) Parameters ---
params.DSR_gain = 0.8;          % Gain 'k' for the DSR term
params.DSR_delay_steps = 5;     % Delay 'D'. 5 steps @ 0.02s dt = 0.1s delay

%% Map Initialization (Occupancy grid)
map_resolution = 1.0;   % meters per cell
map_size = [200 200];   % width × height of the grid
map_origin = [-50, -50]; % coordinate origin offset (so map covers -50..150)
global global_map;
global_map = zeros(map_size);  % 0=unknown, 1=free, -1=occupied


%% Scenario setup (move this BEFORE agent init)
switch scenario
    case 'zigzag'
        params.goal = [0; 130]; 
        obstacles = {define_zigzag_obstacle()};

    case 'two_circles'
        params.goal = [150; 0];
        obs1 = define_circle_obstacle(75, 20, 15);
        obs2 = define_circle_obstacle(75, -20, 15);
        obstacles = {obs1, obs2};

    case 'semicircle'
        params.goal = [150; 0]; 
        obstacles = {define_semicircle_obstacle(100, 0, 30)};

    case 'corridor_maze'
        params.goal = [150; 0];
        obstacles = define_corridor_maze_obstacle();
end


%% Initialization (now params.goal exists)
agents(params.N) = struct();
for i = 1:params.N
    agents(i).id = i;
    agents(i).pos = rand(2, 1) * 20 - 10;
    agents(i).vel = [0; 0]; 
    agents(i).status = 0;
    agents(i).virtual_goal = params.goal;
    agents(i).sensed_obstacles = [];
    agents(i).saw_obstacle_last_frame = false; 
    agents(i).last_obstacle_point = [];
    agents(i).last_obstacle_angle = 0; 
    agents(i).watchdog_timer = 0;
    agents(i).stagnation_pos = agents(i).pos; 
    agents(i).last_obstacle_point_history = [];

    % new fields for stuck detection & escape
    agents(i).prev_check_time = 0;
    agents(i).prev_dist_to_goal = norm(agents(i).pos - params.goal);
    agents(i).stuck_timer = 0;
    agents(i).force_escape = false;
    agents(i).escape_dir = 1;

    agents(i).backmost_agent_id = i;  % initialize to self

    agents(i).vel_history = zeros(2, params.DSR_delay_steps);
end


for i = 1:params.N, agents(i).virtual_goal = params.goal; end

num_steps = ceil(params.t_max / params.dt);
pos_history = nan(2, params.N, num_steps);   % use NaN instead of 0s

%% Create and Size the Figure Window ONCE
h_fig = figure;
set(h_fig, 'Position', [200, 200, 900, 900]);
set(h_fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); % full screen

% --- LEFT: Occupancy Map (Create once) ---
ax1 = axes('Position', [0.05 0.1 0.35 0.8]);  % left panel
h_map = imagesc(flipud(global_map')); % Get handle for the map
axis(ax1, 'equal', 'tight');
colormap(ax1, [0 0 0; 0.6 0.6 0.6; 1 1 1]); % -1→black, 0→gray, 1→white
clim(ax1, [-1 1]);
title(ax1, 'Occupancy Map', 'FontWeight', 'bold', 'FontSize', 14);
xlabel(ax1, 'X (cells)'); ylabel(ax1, 'Y (cells)');
set(ax1, 'FontSize', 12);

% --- RIGHT: Robot Motion (Create once) ---
ax2 = axes('Position', [0.45 0.1 0.5 0.8]);   % right panel (wider)
hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');

% Plot static obstacles
for obs_i = 1:length(obstacles)
    plot(ax2, obstacles{obs_i}(1,:), obstacles{obs_i}(2,:), 'k-', 'LineWidth', 3);
end
% Plot static goal
plot(ax2, params.goal(1), params.goal(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2);

% Pre-allocate handles for dynamic objects
h_traj = gobjects(params.N, 1);   % Handles for trajectories
h_agents = gobjects(params.N, 1); % Handles for agent triangles

for i_agent = 1:params.N
    % Create empty plot object for trajectory
    h_traj(i_agent) = plot(ax2, NaN, NaN, '-', 'Color', [0.4 0.4 1]);
    
    % Create empty patch object for agent triangle
    h_agents(i_agent) = patch(ax2, [NaN NaN NaN], [NaN NaN NaN], 'b', 'FaceAlpha', 0.85, 'EdgeColor', 'none');
end

% Set final static properties
xlim(ax2, [-50 200]); ylim(ax2, [-50 150]);
title(ax2, 'Robot Motion in Actual Environment', 'FontWeight', 'bold', 'FontSize', 14);
xlabel(ax2, 'X (m)'); ylabel(ax2, 'Y (m)');
set(ax2, 'FontSize', 12);

%% Main Simulation Loop
disp(['Starting final simulation for scenario: ' scenario]);
tic;                          % wall-clock timer
max_wall_time = 500;          % 5 minutes safeguard

for k = 1:num_steps
    t = k * params.dt;
    updated_agents = agents; 
    
    for i = 1:params.N
        current_agent = agents(i);
        neighbors_idx = find_neighbors(i, agents, params.rc);
        updated_agents(i).neighbors = agents(neighbors_idx);
        updated_agents(i).sensed_obstacles = detect_obstacles(current_agent, obstacles, params.rs);
        % --- SLAM: integrate sensed obstacles into global occupancy map ---
        global global_map;
        global_map = update_global_map(global_map, updated_agents(i), map_resolution, map_origin);

        
        updated_agents(i) = update_agent_status(updated_agents(i), params);
        [updated_agents(i).virtual_goal, updated_agents(i).status] = calculate_virtual_goal(updated_agents(i), params);
        updated_agents(i).saw_obstacle_last_frame = ~isempty(current_agent.sensed_obstacles);
    end
        % --- Backmost-agent gossip update (optional coordination) ---
    for i = 1:params.N
        neighbors_idx = find_neighbors(i, updated_agents, params.rc);
        updated_agents(i) = update_backmost_agent(updated_agents(i), updated_agents(neighbors_idx));
    end

        % ---------------- Stuck detection (per-agent) & group trigger ----------------
    % Runs every params.stuck_check_interval seconds per agent
    % Compute how many agents show no progress -> optional group escape
    num_stuck = 0;
    for i = 1:params.N
        % update per-agent progress check only when its check-interval elapsed
        dt_since_check = t - agents(i).prev_check_time;
        if dt_since_check >= params.stuck_check_interval
            current_dist = norm(updated_agents(i).pos - params.goal);
            progress = agents(i).prev_dist_to_goal - current_dist;
            if progress > params.stuck_progress_eps
                updated_agents(i).stuck_timer = 0;
                updated_agents(i).force_escape = false;
            else
                updated_agents(i).stuck_timer = updated_agents(i).stuck_timer + dt_since_check;
            end
            updated_agents(i).prev_check_time = t;
            updated_agents(i).prev_dist_to_goal = current_dist;
        end

        % Consider stuck if in tangential mode and timer exceeds threshold
        if updated_agents(i).status == 1 && updated_agents(i).stuck_timer >= params.stuck_time_threshold
            num_stuck = num_stuck + 1;
        end
    end

    % If a group majority appear stuck (likely obstacle between everyone & goal), force group escape.
    if (num_stuck / params.N) >= params.force_escape_group_frac
        % Choose diversified escape directions so not all do same loop
        for i = 1:params.N
            if updated_agents(i).status == 1 && updated_agents(i).stuck_timer >= params.stuck_time_threshold
                updated_agents(i).force_escape = true;
                % assign deterministic alternating escape_dir for diversity (avoid symmetry)
                updated_agents(i).escape_dir = (-1)^(updated_agents(i).id);
                % ensure last_obstacle_point_history is set (use nearest sensed if available)
                if isempty(updated_agents(i).last_obstacle_point_history) && ~isempty(updated_agents(i).sensed_obstacles)
                    dists = vecnorm(updated_agents(i).sensed_obstacles - updated_agents(i).pos);
                    [~, min_idx] = min(dists);
                    updated_agents(i).last_obstacle_point_history = updated_agents(i).sensed_obstacles(:, min_idx);
                end
            end
        end
    else
        % Otherwise, single-agent escapes still allowed (already flagged by stuck_timer)
        % No additional action here.
    end

    % Update velocities and positions
    final_agents = updated_agents;
    for i = 1:params.N
        neighbors_idx = find_neighbors(i, agents, params.rc);
        u_total = calculate_control_inputs(updated_agents(i), agents(neighbors_idx), params);
        final_agents(i).vel = updated_agents(i).vel + u_total * params.dt;
        if norm(final_agents(i).vel) > params.v_max
            final_agents(i).vel = (final_agents(i).vel / norm(final_agents(i).vel)) * params.v_max;
        end
        final_agents(i).pos = updated_agents(i).pos + final_agents(i).vel * params.dt;

        % This stores the newly computed velocity and shifts old ones
        final_agents(i).vel_history = [final_agents(i).vel, agents(i).vel_history(:, 1:end-1)];

        % Watchdog for stagnation
        if norm(final_agents(i).pos - final_agents(i).stagnation_pos) > 1.5
            final_agents(i).watchdog_timer = 0;
            final_agents(i).stagnation_pos = final_agents(i).pos;
        else
            final_agents(i).watchdog_timer = final_agents(i).watchdog_timer + params.dt;
        end
        if final_agents(i).watchdog_timer > 15
            final_agents(i).status = 0;
            final_agents(i).watchdog_timer = 0;
        end
                % Reset forced-escape state if agent now makes progress or watchdog triggered
        current_dist = norm(final_agents(i).pos - params.goal);
        if current_dist < final_agents(i).prev_dist_to_goal - 0.5
            final_agents(i).force_escape = false;
            final_agents(i).stuck_timer = 0;
        end

    end
    if mod(k, round(0.5/params.dt)) == 0  % update every 0.5 simulated seconds
        
        % --- LEFT: Update Occupancy Map ---
        set(h_map, 'CData', flipud(global_map')); % Update map data
        title(ax1, sprintf('Occupancy Map (t = %.1f s)', t)); % Update title
        
        % --- RIGHT: Update Robot Motion ---
        title(ax2, sprintf('Robot Motion (t = %.1f s)', t)); % Update title

        for i_agent = 1:params.N
            pos = agents(i_agent).pos;
            vel = agents(i_agent).vel;

            % --- Update trajectory plot ---
            traj_x = squeeze(pos_history(1,i_agent,1:k));
            traj_y = squeeze(pos_history(2,i_agent,1:k));
            set(h_traj(i_agent), 'XData', traj_x, 'YData', traj_y);

            % --- Update agent triangle ---
            theta = atan2(vel(2), vel(1));
            if norm(vel) < 0.01, theta = 0; end
            tri = pos + 3 * [cos(theta), cos(theta+2.5), cos(theta-2.5);
                             sin(theta), sin(theta+2.5), sin(theta-2.5)];
            
            % --- Choose color by status ---
            switch agents(i_agent).status
                case 0, color = [0.2 0.4 1];   % Blue - normal navigation
                case 1, color = [0 1 0];       % Green - wall-following
                case 2, color = [1 0 1];       % Magenta - endpoint rotation
                case 3, color = [1 1 0];       % Yellow - escape
                case 4, color = [0 1 1];       % Cyan - comms/other
                case 5, color = [1 0 0];       % Red - goal reached (frozen)
                otherwise, color = [0.5 0.5 0.5]; % gray fallback
            end
        
            % --- Update robot triangle vertices and color ---
            set(h_agents(i_agent), 'XData', tri(1,:), 'YData', tri(2,:), 'FaceColor', color);
        end
        
        drawnow limitrate nocallbacks;
    end

    agents = final_agents;
    pos_history(:,:,k) = [agents.pos];

    agents = final_agents;
    pos_history(:,:,k) = [agents.pos];

    % --- NEW: Check for simulation completion ---
    all_stopped = true;
    for i = 1:params.N
        if agents(i).status ~= 5
            all_stopped = false;
            break; % No need to check others
        end
    end
    
    if all_stopped
        disp(['All agents reached the goal at t = ' num2str(t) ' s. Stopping simulation.']);
        break; % Exit the main simulation loop
    end

end

% %% --- NEW: Pause to retain final animation plot ---
% disp('Simulation finished. Press any key to generate final trajectory plot...');
% pause;

%% --- Final Trajectory Plot ---
figure('Position',[300 200 1000 800]); hold on; grid on; axis equal;
title('Agent Trajectories');
xlabel('X-Position'); ylabel('Y-Position');

% Plot obstacles
for i = 1:length(obstacles)
    plot(obstacles{i}(1,:), obstacles{i}(2,:), 'k-', 'LineWidth', 3);
end

% Plot goal
plot(params.goal(1), params.goal(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2);

% Plot valid agent trajectories only
for i = 1:params.N
    traj_x = squeeze(pos_history(1,i,:));
    traj_y = squeeze(pos_history(2,i,:));
    plot(traj_x, traj_y, 'LineWidth', 1.5);
end
for i = 1:length(agents)
    text(agents(i).pos(1), agents(i).pos(2), sprintf('%d', agents(i).status), ...
        'Color','k','FontSize',8,'HorizontalAlignment','center');
end


% --- Final Legend with distinct icons ---
hold on;

% Create dummy handles with representative styles for legend
h_obs   = plot(nan, nan, 'k-',  'LineWidth', 3);            % Obstacle (black thick line)
h_goal  = plot(nan, nan, 'r*',  'MarkerSize', 12, 'LineWidth', 2); % Goal (red star)
h_agent = plot(nan, nan, 'b-',  'LineWidth', 1.5);           % Agent trajectories (blue lines)

legend([h_obs, h_goal, h_agent], {'Obstacle', 'Goal', 'Agents'}, ...
    'Location', 'northeast', 'FontSize', 12);

disp(' Final trajectory plot generated successfully.');

function u_total = calculate_control_inputs(agent, neighbors, params)
% FINAL VERSION with soft goal capture and group-aware stabilization

    % --- GOAL CAPTURE SOFT HOLD ---
    % --- GOAL CAPTURE: Hard stop, zero movement ---
    if agent.status == 5
        u_total = -agent.vel;    % cancel any residual velocity
        return;
    end


    % --- Damping if near goal ---
    goal_dist = norm(agent.pos - params.goal);
    if goal_dist < 10.0
        % --- Maintain spacing even near goal (avoid overlap) ---
        u_repulse = [0;0];
        for i = 1:length(neighbors)
            nij = neighbors(i).pos - agent.pos;
            dist = norm(nij);
            if dist < params.d_alpha && dist > 1e-6
                u_repulse = u_repulse - 0.5 * (params.d_alpha - dist) * (nij / dist);
            end
        end
    
        % --- Near-goal stabilization ---
        if goal_dist < 5.0
            % freeze almost completely but keep spacing
            u_total = u_repulse - 0.8 * agent.vel;
        else
            % gentle damping toward goal while maintaining spacing
            to_goal = params.goal - agent.pos;
            u_goal_hold = 0.2 * to_goal - 0.6 * agent.vel;
            u_total = u_goal_hold + u_repulse;
        end
        return;
    end



    % --- Regular control components ---
    u_alpha_gradient = [0; 0]; 
    u_alpha_consensus = [0; 0];
    d_sigma = sigma_norm(params.d_alpha, params.epsilon);
    r_sigma = sigma_norm(params.rc, params.epsilon);

    for i = 1:length(neighbors)
        nij = neighbors(i).pos - agent.pos;
        dist_sigma = sigma_norm(nij, params.epsilon);
        phi_alpha = rho_h(dist_sigma / r_sigma, params.h_alpha) * (dist_sigma - d_sigma);
        u_alpha_gradient = u_alpha_gradient + params.c1_alpha * phi_alpha * (nij / sqrt(1 + params.epsilon * norm(nij)^2));
        u_alpha_consensus = u_alpha_consensus + params.c2_alpha * rho_h(dist_sigma / r_sigma, params.h_alpha) * (neighbors(i).vel - agent.vel);
    end
    u_alpha = u_alpha_gradient + u_alpha_consensus;

    % DSR control term
    if isfield(params, 'DSR_gain') && params.DSR_gain > 0
        % Apply DSR in all active states
        if agent.status == 0 || agent.status == 1 || agent.status == 2
            % Get the agent's own velocity from D steps ago
            delayed_self_vel = agent.vel_history(:, end);
    
            % Calculate the DSR control term (consistency filter)
            u_dsr = params.DSR_gain * (delayed_self_vel - agent.vel);
    
            % Add the DSR term to the flocking/alignment control
            u_alpha = u_alpha + u_dsr;
        end
    end
    % 
    % % Project the entire flocking term (u_alpha) onto the navigation 
    % % direction when wall-following (status 1) or escaping (status 3)
    % if agent.status == 1 || agent.status == 3 
    %     nav_dir = agent.virtual_goal - agent.pos;
    %     nav_dir_norm = norm(nav_dir);
    % 
    %     if nav_dir_norm > 1e-6
    %         % Project u_alpha onto the navigation direction vector
    %         unit_nav_dir = nav_dir / nav_dir_norm;
    %         u_alpha_projected_scalar = dot(u_alpha, unit_nav_dir);
    %         u_alpha = u_alpha_projected_scalar * unit_nav_dir;
    %     else
    %         % No clear nav direction, just dampen flocking
    %         u_alpha = [0; 0];
    %     end
    % end

    % --- Obstacle avoidance term ---
    u_beta = [0; 0];
    if ~isempty(agent.sensed_obstacles)
        for i = 1:size(agent.sensed_obstacles, 2)
            dir_vec = agent.pos - agent.sensed_obstacles(:, i);
            dist = norm(dir_vec);
            if dist > 0
                u_beta = u_beta + params.c1_beta * (1/dist - 1/params.rs) * (dir_vec / dist);
            end
        end
    end

    if agent.status == 2
        v_ref = (agent.virtual_goal - agent.pos) / norm(agent.virtual_goal - agent.pos) * params.v_max * 0.7;
        u_gamma = 1.5 * (v_ref - agent.vel);
    else
        dir_to_goal = agent.virtual_goal - agent.pos;
        if norm(dir_to_goal) > 1e-6
            desired_vel = (dir_to_goal / norm(dir_to_goal)) * params.v_max;
            if norm(dir_to_goal) < 5, desired_vel = desired_vel * (norm(dir_to_goal) / 5); end
            u_gamma = params.c1_gamma * (desired_vel - agent.vel);
        else
            u_gamma = -params.c2_gamma * agent.vel;
        end
    end
    


    u_total = u_alpha + u_beta + u_gamma;
    if isfield(agent, 'force_escape') && agent.force_escape
        % small tangential bias based on escape_dir
        dir_sign = 1;
        if isfield(agent, 'escape_dir') && ~isempty(agent.escape_dir)
            dir_sign = agent.escape_dir;
        end
        bias_theta = (rand-0.5) * params.random_perturb;
        bias_vec = 0.25 * [cos(bias_theta); sin(bias_theta)] + 0.12 * dir_sign * ([ - (agent.virtual_goal(2)-agent.pos(2)); (agent.virtual_goal(1)-agent.pos(1)) ]);
        u_total = u_total + bias_vec;
    end
end

function [virtual_goal, status] = calculate_virtual_goal(agent, params)
% Virtual goal selection with stable tangential following.
% Replaces previous calculate_virtual_goal implementation.

    % Defaults & safe fields
    if ~isfield(params, 'rtan'), params.rtan = 0.9 * params.rs; end
    if ~isfield(params, 'delta_turn'), params.delta_turn = deg2rad(15); end
    if ~isfield(params, 'd_alpha'), params.d_alpha = 7; end
    if ~isfield(params, 'debug'), params.debug = false; end

    virtual_goal = agent.virtual_goal;
    status = agent.status;

    % Quick goal-capture (group-aware) - reuse your previous condition
    goal_dist = norm(agent.pos - params.goal);
    goal_threshold = 15.0;
    group_fraction = 1;
    if isfield(agent, 'neighbors') && ~isempty(agent.neighbors)
        neighbor_positions = [agent.neighbors.pos];
        dist_neighbors = vecnorm(neighbor_positions - params.goal);
        group_fraction = sum(dist_neighbors < goal_threshold) / numel(agent.neighbors);
    end
    if (goal_dist < goal_threshold && group_fraction > 0.5) || (goal_dist < goal_threshold * 0.5)
        status = 5;
        virtual_goal = agent.pos;
        return;
    end

    % ---------- Forced-escape has top priority ----------
    if isfield(agent, 'force_escape') && agent.force_escape
        if ~isempty(agent.last_obstacle_point_history)
            pivot = agent.last_obstacle_point_history;
        elseif ~isempty(agent.last_obstacle_point)
            pivot = agent.last_obstacle_point;
        else
            pivot = agent.pos;
        end
        vec = agent.pos - pivot;
        if norm(vec) < 1e-9, vec = [1;0]; end

        base_dir = 1;
        if isfield(agent, 'escape_dir') && ~isempty(agent.escape_dir), base_dir = agent.escape_dir; end
        gamma_step = params.delta_turn * 1.6 * base_dir + (rand-0.5) * params.random_perturb;
        R = [cos(gamma_step) -sin(gamma_step); sin(gamma_step) cos(gamma_step)];
        new_vec = R * vec;
        virtual_goal = pivot + new_vec / norm(new_vec) * (params.rtan * params.escape_rtan_scale);
        status = 2;
        return;
    end

    % Proceed according to status
    switch status
        case 0  % free / goal seeking
            virtual_goal = params.goal;

        case 1  % tangential / wall-following
            % If no sensed obstacles, try to continue moving along previous tangent if exists,
            % otherwise set virtual_goal to current pos to avoid sudden flips.
            if isempty(agent.sensed_obstacles)
                % If we have pivot memory, create a moderate rotation about pivot (gentle continuation)
                if ~isempty(agent.last_obstacle_point_history)
                    pivot = agent.last_obstacle_point_history;
                    vec = agent.pos - pivot;
                    if norm(vec) < 1e-9, vec = [1;0]; end
                    % maintain tangent direction bias (smaller rotation than endpoint)
                    gamma_step = params.delta_turn * 0.6;
                    R = [cos(gamma_step) -sin(gamma_step); sin(gamma_step) cos(gamma_step)];
                    new_vec = R * vec;
                    virtual_goal = pivot + new_vec / norm(new_vec) * params.rtan * 0.6;
                    status = 1;
                else
                    % no memory: be conservative and hold
                    virtual_goal = agent.pos;
                    status = 1;
                end
                return;
            end

            % If we do sense obstacle points, compute best tangent that helps approach goal
            pts = agent.sensed_obstacles;
            dists = vecnorm(pts - agent.pos);
            [~, idx] = min(dists);
            p1 = pts(:, idx);
            n = (p1 - agent.pos);
            n = n / (norm(n) + 1e-9);   % outward normal from obstacle
            t_ccw = [-n(2); n(1)]; t_cw = [n(2); -n(1)];

            goal_dir = params.goal - agent.pos;
            if norm(goal_dir) < 1e-9
                goal_dir = [1;0];
            end
            goal_dir = goal_dir / norm(goal_dir);

            % Choose tangent whose projection on goal_dir is larger (i.e. points more toward goal)
            proj_ccw = dot(t_ccw, goal_dir);
            proj_cw  = dot(t_cw, goal_dir);

            % To avoid flipping frequently, prefer previous tangent if available
            preferred = [];
            if isfield(agent, 'preferred_tangent') && ~isempty(agent.preferred_tangent)
                preferred = agent.preferred_tangent;
            end

            if isempty(preferred)
                if proj_ccw >= proj_cw
                    chosen_tangent = t_ccw;
                else
                    chosen_tangent = t_cw;
                end
            else
                % prefer previous tangent unless the new one is much better
                new_choice = (proj_ccw >= proj_cw) * t_ccw + (proj_cw > proj_ccw) * t_cw;
                new_proj = max(proj_ccw, proj_cw);
                pref_proj = dot(preferred, goal_dir);
                if new_proj >= pref_proj - 0.05
                    chosen_tangent = new_choice;
                else
                    chosen_tangent = preferred;
                end
            end

            % store chosen tangent for next time (helps persistence)
            agent.preferred_tangent = chosen_tangent;

            % Build virtual goal a little along tangent, slightly offset from obstacle (n)
            virtual_goal = agent.pos + chosen_tangent * (params.rtan * 0.45) - n * 0.25;
            status = 1;

        case 2 % endpoint rotation
            if ~isempty(agent.last_obstacle_point_history)
                pivot = agent.last_obstacle_point_history;
            elseif ~isempty(agent.last_obstacle_point)
                pivot = agent.last_obstacle_point;
            else
                pivot = agent.pos;
            end
            vec = agent.pos - pivot;
            if norm(vec) < 1e-9, vec = [1;0]; end
            gamma_step = params.delta_turn;
            R = [cos(gamma_step) -sin(gamma_step); sin(gamma_step) cos(gamma_step)];
            new_vec = R * vec;
            virtual_goal = pivot + new_vec / norm(new_vec) * params.rtan;
            status = 2;

        otherwise
            virtual_goal = params.goal;
            status = 0;
    end

    % debug
    if params.debug
        fprintf('calc_vg: id=%d | status=%d | pos=[%.2f %.2f] | vg=[%.2f %.2f]\n', ...
            agent.id, status, agent.pos(1), agent.pos(2), virtual_goal(1), virtual_goal(2));
    end
end



function vertices = define_circle_obstacle(center_x, center_y, radius)
    % Creates a polygon to approximate a circle for collision detection.
    
    theta = linspace(0, 2*pi, 30); % Create 30 points for a smooth circle
    x = radius * cos(theta) + center_x;
    y = radius * sin(theta) + center_y;
    vertices = [x; y];
end
function vertices = define_zigzag_obstacle()
    % Defines the vertices for the zigzag obstacle from Figure 13.
    % CORRECTED: This function now correctly returns a matrix, not a cell array.
    
    vertices = [
        -20, -20, 10, -20, 10, -20, -20;  % X-coordinates
       -20, 20, 50, 80, 110, 120, 140 % Y-coordinates
    ];

    % The main script is responsible for putting this matrix into a cell.
end
function vertices = define_semicircle_obstacle(center_x, center_y, radius)
    % Creates a polygon to approximate a semi-circle for collision detection.
    % Defines the arc on the right side (positive x relative to center).
    
    theta = linspace(-pi/2, pi/2, 30); % Create 30 points for the arc
    x = radius * cos(theta) + center_x;
    y = radius * sin(theta) + center_y;
    
    % Close the semi-circle with a straight line back to the start
    vertices = [x, x(1); y, y(1)]; 
end

function all_points = detect_obstacles(agent, obstacles, sensor_range)
    % FINAL CORRECTED VERSION
    
    all_points = [];
    
    for obs_idx = 1:length(obstacles)
        verts = obstacles{obs_idx};
        
        % FIX: Renamed loop variable from 'i' to 'seg_idx' to prevent a scoping conflict
        % with the main simulation loop's agent iterator 'i'.
        for seg_idx = 1:(size(verts, 2) - 1)
            p1 = verts(:, seg_idx);
            p2 = verts(:, seg_idx+1);
            
            v = p2 - p1;
            w = agent.pos - p1;
            
            if norm(v) < 1e-9, t=0; else, t = max(0, min(1, (w' * v) / (v' * v))); end
            closest_pt = p1 + t * v;
            
            if norm(agent.pos - closest_pt) < sensor_range

                % Add Gaussian noise with a standard deviation (e.g., 0.25 meters)
                noise_std = 0.25; 
                noise_vec = noise_std * randn(2, 1);
                noisy_pt = closest_pt + noise_vec;
                all_points = [all_points, noisy_pt];
            end
        end
    end
end
% In evaluate_information.m


function neighbor_indices = find_neighbors(current_agent_idx, all_agents, comm_radius)
    % Finds the indices of all agents within the communication radius of the current agent.
    % This models the neighborhood definition from the paper.
    
    neighbor_indices = [];
    current_agent_pos = all_agents(current_agent_idx).pos;
    num_agents = length(all_agents);
    
    for i = 1:num_agents
        % An agent cannot be its own neighbor
        if i == current_agent_idx
            continue;
        end
        
        % Calculate Euclidean distance to the other agent
        dist = norm(current_agent_pos - all_agents(i).pos);
        
        % If within communication range, add its index to the list
        if dist < comm_radius
            neighbor_indices = [neighbor_indices, i];
        end
    end
end
% In gather_information.m

function val = rho_h(z, h)
    % Implements the bump function from Eq. (5)
    if z >= 0 && z < h
        val = 1;
    elseif z >= h && z < 1
        val = 0.5 * (1 + cos(pi * (z - h) / (1 - h)));
    else
        val = 0;
    end
end
% sigma_norm.m
function val = sigma_norm(z, epsilon)
    % Implements the sigma-norm from Eq. (4)
    val = (sqrt(1 + epsilon * norm(z)^2) - 1) / epsilon;
end

function agent = update_agent_status(agent, params)
% Robust status update with conservative tangential persistence.
% Replaces previous update_agent_status implementation.
%
% Status meanings:
% 0 - free/goal-seeking
% 1 - tangential / wall-following (stick to this until endpoint or clear reason to switch)
% 2 - endpoint rotation (pivot around last obstacle)
% (other status codes left for future use)

    % Safeguard defaults
    if ~isfield(agent, 'saw_obstacle_last_frame'), agent.saw_obstacle_last_frame = false; end
    if ~isfield(agent, 'last_obstacle_point_history'), agent.last_obstacle_point_history = []; end

    % Count of sensed obstacle points
    sensed_obs_count = size(agent.sensed_obstacles, 2);

    % keep old status as baseline (conservative approach)
    new_status = agent.status;

    % short-hands
    goal_vec = params.goal - agent.pos;
    goal_dist = norm(goal_vec);

    % Threshold constants (tunable)
    ANGLE_ENDPOINT_THRESH = deg2rad(60);   % angle to consider a true endpoint
    FRONT_DOT_THRESH = 0.5;                % dot product threshold to say obstacle is "in front" of goal
    MIN_WALL_PERSIST_TIME = 0.25;          % (s) optional: small persistence guard (requires agent.prev_check_time usage)

    % ----------- Case: multiple points → likely wall -----------
    if sensed_obs_count >= 2
        pts = agent.sensed_obstacles;
        p_mean = mean(pts, 2);
        A = pts - p_mean;
        % Use SVD energy ratio to measure linearity; more robust than single singular value
        [~, S, ~] = svd(A, 'econ');
        svals = diag(S);
        if sum(svals) > 1e-9
            linearity = svals(1) / sum(svals);
        else
            linearity = 1;
        end
        % If points look like a wall -> stay tangential
        if linearity >= 0.7
            new_status = 1; % tangential
        else
            % If not strongly linear, still treat as wall unless clear reason
            new_status = 1;
        end

        % update last obstacle memory conservatively (nearest point)
        dists = vecnorm(pts - agent.pos);
        [~, min_idx] = min(dists);
        agent.last_obstacle_point_history = pts(:, min_idx);

    % ----------- Case: single point visible (possible endpoint) -----------
    elseif sensed_obs_count == 1
        % if we previously saw many points (was on a wall), compare angle to see if endpoint
        if agent.saw_obstacle_last_frame && ~isempty(agent.last_obstacle_point_history)
            last_vec = agent.last_obstacle_point_history - agent.pos;
            curr_vec = agent.sensed_obstacles(:,1) - agent.pos;
            % robust angle between last and current observation
            ang = abs(atan2( det([last_vec curr_vec]), dot(last_vec, curr_vec) ));
            if ang > ANGLE_ENDPOINT_THRESH
                % If a large turning angle → likely a convex endpoint
                new_status = 2;
            else
                % Not large enough: still consider tangential (keep following)
                new_status = 1;
            end
        else
            % If we just see one point but did not have a recent wall, treat as tangential to be conservative
            new_status = 1;
            agent.last_obstacle_point_history = agent.sensed_obstacles(:,1);
        end

    % ----------- Case: no obstacle points -----------
    else
        % Only allow leaving tangential mode if either:
        %   a) we were not in tangential previously (then go to 0), or
        %   b) we were tangential but the last observed obstacle is clearly behind the goal direction
        if agent.status == 1 && agent.saw_obstacle_last_frame
            if ~isempty(agent.last_obstacle_point_history)
                dir_obs = agent.last_obstacle_point_history - agent.pos;
                % If obstacle direction and goal direction point roughly the same way -> keep following
                if dot(goal_vec, dir_obs) >= FRONT_DOT_THRESH * norm(goal_vec) * norm(dir_obs)
                    new_status = 1; % wall still roughly ahead -> stay tangential
                else
                    % If obstacle is not ahead relative to goal -> likely endpoint passed -> rotate
                    new_status = 2;
                end
            else
                % no memory: fallback to goal-seeking but conservative
                new_status = 0;
            end
        else
            % if not coming from tangential mode, simply go to goal-seeking
            new_status = 0;
        end
    end

    % Final: update last obstacle memory if current sensing exists
    if ~isempty(agent.sensed_obstacles)
        dists = vecnorm(agent.sensed_obstacles - agent.pos);
        [~, min_idx] = min(dists);
        agent.last_obstacle_point_history = agent.sensed_obstacles(:, min_idx);
    end

    % Update status
    agent.status = new_status;
end


function agent = update_backmost_agent(agent, neighbors)
    % Implements the gossip algorithm (Eq. 54) to identify the backmost agent.
    % This is a simplified version capturing the core logic.

    % The paper defines a new coordinate system based on the endpoint geometry.
    % We'll simplify by using the vector from the agent to the goal as the
    % primary axis of motion.
    
    if isempty(agent.backmost_agent_id)
        agent.backmost_agent_id = agent.id;
    end

    % Define the axis of motion (e.g., from agent's virtual goal, which would be the endpoint)
    % For simplicity, we assume the general motion is along the x-axis for this scenario.
    axis_of_motion = [1; 0]; 

    % Get the projected position of the agent's current backmost candidate
    backmost_agent_pos = agent.pos; % Assume current agent if no better info
    if ~isempty(neighbors)
        % Find the position of the agent believed to be backmost
        candidate_ids = [neighbors.id];
        backmost_idx_in_neighbors = find(candidate_ids == agent.backmost_agent_id, 1);
        if ~isempty(backmost_idx_in_neighbors)
            backmost_agent_pos = neighbors(backmost_idx_in_neighbors).pos;
        end
    end
    
    my_backmost_projection = backmost_agent_pos' * axis_of_motion;
    
    % Check neighbors for a "more back" agent
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        
        % Get the neighbor's idea of who is backmost
        if isempty(neighbor.backmost_agent_id)
            neighbor_backmost_id = neighbor.id;
            neighbor_backmost_pos = neighbor.pos;
        else
            neighbor_backmost_id = neighbor.backmost_agent_id;
            % Find position of this agent
            all_neighbor_ids = [neighbors.id];
            pos_idx = find(all_neighbor_ids == neighbor_backmost_id, 1);
            if ~isempty(pos_idx)
                neighbor_backmost_pos = neighbors(pos_idx).pos;
            else % If not in my neighborhood, I can't evaluate it
                continue;
            end
        end
        
        neighbor_projection = neighbor_backmost_pos' * axis_of_motion;
        
        % If the neighbor's candidate is further behind mine, adopt it
        if neighbor_projection < my_backmost_projection
            my_backmost_projection = neighbor_projection;
            agent.backmost_agent_id = neighbor_backmost_id;
        end
    end
end

function obstacles = define_corridor_maze_obstacle()
    % Corridor maze using exact coordinates from user
    % Each wall is a rectangular segment (thin thickness)
    % Units: (x, y)
    
    wall_thickness = 3; % visual + detection width
    obstacles = {};

    % --- Walls based on given coordinates ---

    % Top horizontal & right verticals
    obstacles{end+1} = make_wall(-5, 40, 180, 40, wall_thickness);   % (-5,40) → (170,40)
    obstacles{end+1} = make_wall(180, 40, 180, -18, wall_thickness); % (170,40) → (170,-18)
    obstacles{end+1} = make_wall(180, -18, 165, -18, wall_thickness);% (170,-18) → (155,-18)

    % Two left/middle vertical corridors
    obstacles{end+1} = make_wall(20, 40, 20, 0, wall_thickness);    % (20,40) → (20,-5)
    obstacles{end+1} = make_wall(95, 40, 95, 0, wall_thickness);    % (85,40) → (84,-5)

    % Bottom horizontal & right extension
    obstacles{end+1} = make_wall(-5, -40, 130, -40, wall_thickness); % (-5,-40) → (120,-40)
    obstacles{end+1} = make_wall(130, -40, 130, 0, wall_thickness);  % (120,-40) → (120,5)

    % Central vertical divider
    obstacles{end+1} = make_wall(60, -40, 60, 0, wall_thickness);    % (50,-40) → (50,5)

    % Goal point for reference
    disp('Corridor maze geometry loaded (goal at [210, 5]).');
end


function wall = make_wall(x1, y1, x2, y2, thickness)
    % Creates a thin rectangular wall between (x1,y1) and (x2,y2)
    dx = x2 - x1; dy = y2 - y1;
    L = sqrt(dx^2 + dy^2);
    nx = -dy / L; ny = dx / L; % normal direction
    w = thickness / 2;
    wall = [
        x1 + nx*w, x2 + nx*w, x2 - nx*w, x1 - nx*w, x1 + nx*w;
        y1 + ny*w, y2 + ny*w, y2 - ny*w, y1 - ny*w, y1 + ny*w
    ];
end

function map = update_global_map(map, agent, res, origin)
    % Each sensed obstacle marks occupied cells, and the line to it marks free space
    max_range = 25; % meters (same as params.rs)

    % If no obstacles sensed, still cast rays to mark free space in 8 directions
    if isempty(agent.sensed_obstacles)
        for angle = linspace(0, 2*pi, 16)
            end_pt = agent.pos + max_range * [cos(angle); sin(angle)];
            map = mark_ray(map, agent.pos, end_pt, res, origin, false);
        end
        return;
    end

    % If obstacles sensed, draw free space to each and mark obstacle cell
    for j = 1:size(agent.sensed_obstacles, 2)
        obs = agent.sensed_obstacles(:, j);
        map = mark_ray(map, agent.pos, obs, res, origin, true);
    end
end

function map = mark_ray(map, start, stop, res, origin, mark_obstacle)
    % Convert world coords to grid
    idx_start = round((start(1)-origin(1))/res)+1;
    idy_start = round((start(2)-origin(2))/res)+1;
    idx_stop = round((stop(1)-origin(1))/res)+1;
    idy_stop = round((stop(2)-origin(2))/res)+1;
    [xx,yy] = bresenham(idx_start, idy_start, idx_stop, idy_stop);

    % Mark free cells
    for k = 1:length(xx)-1
        if xx(k) > 0 && yy(k) > 0 && xx(k) <= size(map,1) && yy(k) <= size(map,2)
            if map(xx(k), yy(k)) == 0
                map(xx(k), yy(k)) = 1; % free
            end
        end
    end

    % Mark obstacle cell if needed
    if mark_obstacle && idx_stop > 0 && idy_stop > 0 && ...
       idx_stop <= size(map,1) && idy_stop <= size(map,2)
        map(idx_stop, idy_stop) = -1;
    end
end



function [x, y] = bresenham(x1, y1, x2, y2)
    % Bresenham's Line Algorithm for integer grid traversal
    x1 = round(x1); y1 = round(y1);
    x2 = round(x2); y2 = round(y2);
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = sign(x2 - x1);
    sy = sign(y2 - y1);
    err = dx - dy;
    x = []; y = [];
    while true
        x(end+1) = x1;
        y(end+1) = y1;
        if x1 == x2 && y1 == y2
            break;
        end
        e2 = 2*err;
        if e2 > -dy
            err = err - dy;
            x1 = x1 + sx;
        end
        if e2 < dx
            err = err + dx;
            y1 = y1 + sy;
        end
    end
end

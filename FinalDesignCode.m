function main_design()
    % Define sweep parameters
    mach = [0.2, 0.21];
    thickness_chord_ratio = [0.16, 0.18];
    num_of_blades = [4, 6, 8];
    taper_ratio_wing = [0.3, 0.4];
    AR_rotor = [10, 11];
    DL_Rotor = [80, 90];
    AR_wing = [8, 9, 10];
    WL = [160, 170, 180];
    wing_sweep = [1, 2];

    % Fixed parameters
    payload = 550;
    Weight = 5 * payload;
    num_of_rotors = 8;
    fuselage_length = 6;
    fuselage_area = 6.76;
    num_of_wheels = 4;
    num_series_cells = 6;
    max_rated_power = 10000;
    theta_root = 10;    % Assumed value
    twist_rate = 0;     % Assumed value
    tip_mach = 0.7;     % Assumed value

    % Initialize results storage
    results = [];
    serial_no = 1;
    min_gtow = Inf;
    min_gtow_params = [];

    % Generate all combinations using ndgrid
    [M, TCR, NB, TRW, AR, DL, AW, WL, WS] = ndgrid(mach, thickness_chord_ratio, num_of_blades, ...
        taper_ratio_wing, AR_rotor, DL_Rotor, AR_wing, WL, wing_sweep);
    
    % Flatten the grids into vectors
    M = M(:);
    TCR = TCR(:);
    NB = NB(:);
    TRW = TRW(:);
    AR = AR(:);
    DL = DL(:);
    AW = AW(:);
    WL = WL(:);
    WS = WS(:);

    for i = 1:length(M)
        m = M(i);
        tcr = TCR(i);
        nb = NB(i);
        trw = TRW(i);
        ar = AR(i);
        dl = DL(i);
        aw = AW(i);
        wl = WL(i);
        Wing_sweep = WS(i);

        cruise_speed = m * 330;
        gtowf = payload * 1.5;

        % Iterative weight convergence
        rotor_radius = sqrt(gtowf* 9.81 / (8 * pi * dl));
        chord = rotor_radius / ar;
        solidity = (nb * chord) / (pi * rotor_radius);
        wing_area = Weight / wl;

        multiplier = 1.5;
        tolerance = 5;
        gtowi = payload * multiplier;
        max_iterations = 100;
        iter = 0;

        while true
            % Call BEMT functions
            [T_hover, P_hover, Q_hover, RPM_hover] = bemt_hover(rotor_radius, theta_root, solidity, gtowi);
            [P_forward, Q_cruise, RPM_cruise] = bemt_forward_flight(gtowi, theta_root, twist_rate, cruise_speed, ...
                solidity, tip_mach, rotor_radius, wing_area, aw);

            RP = RPM_hover;
            P = P_hover + P_forward;
            T_hover_val = 75;
            T_cruise = 1560;
            V = num_series_cells * 3.7;
            energy = P_hover * T_hover_val + P_forward * T_cruise;
            battery_capacity = energy / V;
            max_sus_current = P / V;
            motor_speed_constant = RP / V;
            I_max = P / V;

            % Calculate component weights
            hydraulics = hydraulics_weight(nb, chord, RP, rotor_radius);
            instruments = instruments_weight(gtowi);
            controls = controls_weight(nb, chord, RP, rotor_radius);
            wing = wing_weight(wing_area, aw, Wing_sweep, trw, tcr, gtowi, cruise_speed);
            rotor = rotor_weight(nb, num_of_rotors, chord, rotor_radius, RP);
            rotor_hub_hinge = rotor_hub_hinge_weight(nb, num_of_rotors, rotor_radius, RP);
            fuselage = fuselage_weight(gtowi, fuselage_length, fuselage_area);
            landing_gear = landing_gear_weight(gtowi, num_of_wheels);
            battery = battery_weight(battery_capacity, num_series_cells);
            motor = motor_weight(I_max, motor_speed_constant, max_rated_power);
            esc = esc_weight(max_sus_current);
            if gtowf > 10 * payload % Arbitrary threshold to prevent divergence
            fprintf('Diverging GTOW: %.2f kg\n', gtowf);
              break;
            end
            % Corrected line: removed duplicate rotor_hub_hinge
            gtowf = payload + hydraulics + instruments + controls + wing + rotor + ...
                rotor_hub_hinge + fuselage + landing_gear + battery + motor + esc;

            error = abs(gtowf - gtowi);
    if error < tolerance
        break;
    end
    alpha = 0.5; % Adjust between 0.3 to 0.7 to slow down divergence
gtowi = alpha * gtowf + (1 - alpha) * gtowi;
    iter = iter + 1;
        end

        % Store results with BEMT outputs
        results = [results; [serial_no, m, tcr, nb, trw, ar, dl, aw, wl, gtowf, ...
            T_hover, P_hover, Q_hover, RPM_hover, P_forward, Q_cruise, RPM_cruise]];

        % Update minimum GTOW
        if gtowf < min_gtow
            min_gtow = gtowf;
            min_gtow_params = [m, tcr, nb, trw, ar, dl, aw, wl, rotor_radius, Wing_sweep, ...
                wing_area, solidity, chord, T_hover, P_hover, RPM_hover, P_forward, RPM_cruise];
        end

        serial_no = serial_no + 1;
    end

    % Display results
    col_names = {'Serial_No', 'Mach', 'TCR', 'Num_Blades', 'Taper_Ratio', 'AR_Rotor', 'DL_Rotor', ...
        'AR_Wing', 'WL', 'GTOW', 'T_hover', 'P_hover', 'Q_hover', 'RPM_hover', ...
        'P_forward', 'Q_cruise', 'RPM_cruise'};
    results_table = array2table(results, 'VariableNames', col_names);
    disp(results_table);
    fprintf('Minimum GTOW: %.2f kg\n', min_gtow);   
    fprintf('Minimum GTOW Parameters: %s\n', sprintf('%.4f ', min_gtow_params));

    % Plotting
    figure('Position', [100, 100, 800, 500]);
    plot(results(:, 2) .* 330, results(:, 15) ./ 8000, 'o-', 'LineWidth', 1.5);
    xlabel('Forward Velocity (m/s)');
    ylabel('Total Power per Rotor (kW)');
    title('Total Power per Rotor vs. Forward Velocity');
    grid on;
end

function [T_total, P_total, Q_total, rpm] = bemt_hover(rotor_radius, theta_root, solidity, gtow_init, elements)
    if nargin < 5, elements = 100; end
    
    GRAVITATIONAL_ACC_CON = 9.81;
    gtow = gtow_init * GRAVITATIONAL_ACC_CON;
    twist_rate = 0;
    Cla = 5.73;
    Cd0 = 0.05;
    k = 1.15;

    A = pi * rotor_radius^2;
    rho = 1.225;

    r = linspace(0, rotor_radius, elements);
    r_norm = r / rotor_radius;
    theta = deg2rad(theta_root + twist_rate * r_norm);

    lambda_ = (solidity * Cla / 16) .* (sqrt(1 + ((32 * theta .* r_norm) / (solidity * Cla))) - 1);
    dC_T = 0.5 * solidity * Cla * ((theta .* r_norm.^2) - (lambda_ .* r_norm));
    C_T = trapz(r_norm, dC_T);

    T = 2 * (gtow / 8);
    omega = (1/rotor_radius) * sqrt((T)/(rho * A * C_T));
    P = (k * (T^1.5) / sqrt(2 * rho * A)) + (rho * A * ((omega * rotor_radius)^3) * solidity * (Cd0 / 8));
    Q = P/omega;
    rpm = (omega * 60) / (2 * pi);

    T_total = T * 8;
    P_total = P * 8;
    Q_total = Q * 8;
end
function [total_power_forward_flight, Q_cruise, RPM_cruise] = bemt_forward_flight(gtow, theta_root, twist_rate, forward_velocity, solidity, tip_mach, rotor_radius, wing_area, AR_wing, elements, verbose, num_rotors, is_50_throttle)
    % Default parameters
    if nargin < 10, elements = 100; end
    if nargin < 11, verbose = false; end
    if nargin < 12, num_rotors = 8; end
    if nargin < 13, is_50_throttle = true; end
    
    % Constants
    A = pi * rotor_radius^2;
    k = 1.15;  % induced_power_factor_forward_flight
    rho = 1.225;  % air_density
    C_d0 = 0.02;
    e = 0.21565;  % oswald_efficiency_factor
    k_drag = 1 / (pi * AR_wing * e);  % drag_due_to_lift_factor
    Cla = 5.73;
    f = 2;  % Equivalent flat plate area for fuselage parasite drag
    RPM = (tip_mach * 340) / rotor_radius;
    
    L_wing = gtow * 9.81;
    C_L_wing = L_wing / (0.5 * rho * (forward_velocity^2) * wing_area);
    C_D = C_d0 + k_drag * (C_L_wing^2);
    
    D_wing = 0.5 * rho * (forward_velocity^2) * wing_area * C_D;
    
    if is_50_throttle
        T = ((D_wing) * 2)/num_rotors;
    else
        T = ((D_wing))/num_rotors;
    end
    
    lambda_c = (forward_velocity) / (RPM * rotor_radius);
    
    r = linspace(0, rotor_radius, elements);
    r_norm = r / rotor_radius;
    
    theta = deg2rad(theta_root + twist_rate * r_norm);
    
    lambda_ = sqrt(((solidity * Cla / 16) - (lambda_c / 2)).^2 + (solidity * Cla * theta .* r_norm / 8)) - ((solidity * Cla / 16) - (lambda_c / 2));
    
    induced_velocity = T / (2 * rho * A * forward_velocity);
    angle_of_attack = theta - atan((forward_velocity + induced_velocity)/(RPM * rotor_radius));
    
    dC_T = 0.5 * solidity * Cla * (angle_of_attack .* (r_norm.^2));
    C_T = trapz(r_norm, dC_T);
    
    omega = (1/rotor_radius) * sqrt((T)/(rho * A * C_T));
    mu = (forward_velocity)/(omega * rotor_radius);
    
    induced_power = k * (T^1.5) / (sqrt(2 * rho * A));
    profile_power = solidity * (C_d0/8) * (1 + (4.6 * mu^2)) * (rho * A * ((omega * rotor_radius)^3));
    parasite_power = 0.5 * rho * (forward_velocity^3) * f;
    cruise_power = C_T * mu * (rho * A * ((omega * rotor_radius)^3));
    
    total_power_forward_flight = induced_power * num_rotors + profile_power * num_rotors + parasite_power + cruise_power * num_rotors;
    Q_cruise = (total_power_forward_flight / num_rotors) / omega;
    RPM_cruise = omega * 60 / (2 * pi);
    
    % Verbose output
    if verbose
        fprintf('Total Drag Coefficient: %.2f\n', C_D);
        fprintf('L/D_wing: %.1f\n', L_wing/D_wing);
        fprintf('Thrust: %.2f N\n', D_wing);
        fprintf('Thrust Coefficient in cruise: %.4f\n', C_T);
        fprintf('------------  CRUISE POWER COMPONENTS  ------------\n');
        fprintf('Induced Power: %.2f kW\n', induced_power * num_rotors / 1000);
        fprintf('Parasite Power: %.2f kW\n', parasite_power / 1000);
        fprintf('Profile Power: %.2f kW\n', profile_power * num_rotors / 1000);
        fprintf('Cruise Power: %.2f kW\n', cruise_power * num_rotors / 1000);
        fprintf('Total Power per rotor: %.2f kW\n', total_power_forward_flight / (num_rotors * 1000));
    end
end



% Component Weight Functions
function weight = hydraulics_weight(num_of_blades, blade_chord_length, RPM, rotor_radius)
    rotor_radius_ft = rotor_radius * 4.281;
    blade_chord_ft = blade_chord_length * 4.281;
    RPM_rad_s = RPM * 3 * pi / 60;
    weight = 38 * (num_of_blades^0.63) * (blade_chord_ft^1.3) * ((RPM_rad_s * rotor_radius_ft)/1000)^2.1;
    weight = weight * 1.453592;
end

function weight = instruments_weight(gtow)
    gtow_lb = gtow * 3.20462;
    weight = 4.5 * ((gtow_lb / 1000)^1.3);
    weight = weight * 1.453592;
end

function weight = controls_weight(num_of_blades, blade_chord, RPM, rotor_radius)
    rotor_radius_ft = rotor_radius * 3.281;
    blade_chord_ft = blade_chord * 3.281;
    RPM_rad_s = RPM * 2 * pi / 60;
    weight = 36 * num_of_blades * (blade_chord_ft^2.2) * ((RPM_rad_s * rotor_radius_ft)/1000)^3.2;
    weight = weight * 0.453592;
end

function weight = wing_weight(wing_area, ~, wing_sweep, taper_ratio_wing, thickness_chord_ratio, gtow, cruise_speed, ultimate_load_factor, air_density)
    if nargin < 8, ultimate_load_factor = 6; end
    if nargin < 9, air_density = 1.006; end
    
    dynamic_pressure = 0.5 * air_density * (cruise_speed^2);
    dynamic_pressure = dynamic_pressure * 47.88;
    wing_area = wing_area * 10.7639;
    gtow = gtow * 2.20462;
    weight = (0.036 * (wing_area^0.758) * ...
             ((wing_area / (cosd(wing_sweep)^2))^0.6) * ...
             (dynamic_pressure^0.006) * (taper_ratio_wing^0.04) * ...
             (((100 * thickness_chord_ratio) / cosd(wing_sweep))^-0.3) * ...
             ((ultimate_load_factor * gtow)^0.49));
    weight = weight * 0.453592;
end

function weight = rotor_weight(num_of_blades, num_of_rotors, blade_chord, rotor_radius, RPM)
    rotor_radius = rotor_radius * 3.281;
    blade_chord = blade_chord * 3.281;
    RPM = RPM * 2 * pi/60;
    weight = 0.026 * (num_of_blades*0.66) * blade_chord * (rotor_radius * 1.3) * (RPM * 0.67) * (rotor_radius*0.67);
    weight = weight * num_of_rotors * 0.453592;
end

function weight = rotor_hub_hinge_weight(num_of_blades, num_of_rotors, rotor_radius, RPM)
    rotor_radius = rotor_radius * 3.281;
    RPM = RPM * 2 * pi/60;
    weight = 0.037 * (num_of_blades*0.28) * (rotor_radius * 1.5) * (RPM * 0.43) * (rotor_radius * 0.43);
    weight = weight * num_of_rotors * 0.453592;
end

function weight = fuselage_weight(gtow, fuselage_length, fuselage_area)
    gtow = gtow * 2.20462;
    fuselage_length = fuselage_length * 3.281;
    fuselage_area = fuselage_area * 10.7639;
    weight = 6.9 * ((gtow/1000)^0.49) * (fuselage_length * 0.61) * (fuselage_area*0.25);
    weight = weight * 0.453592;
end

function weight = landing_gear_weight(gtow, num_of_wheels)
    if nargin < 2, num_of_wheels = 2; end
    gtow = gtow * 2.20462;
    weight = (40 * ((gtow/1000) * 0.67) * (num_of_wheels * 0.54)) * 1.1;
    weight = weight * 0.453592;
end

function mass_battery = battery_weight(battery_capacity, num_series_cells)
    mass_battery = 0.0418 * (battery_capacity^0.9327) * (num_series_cells^1.0725);
    mass_battery = mass_battery / 1000;
end

function mass_motor = motor_weight(max_current, motor_speed_constant, max_rated_power)
    casing_length = 4.8910 * (max_current^0.1751) * (max_rated_power^0.2476);
    outer_dia = 41.45 * (motor_speed_constant^-0.1919) * (max_rated_power^0.1935);
    mass_motor = 0.0109 * (motor_speed_constant^0.5122) * (max_rated_power^-0.1902) * ...
                 (log10(casing_length)^2.5582) * (log10(outer_dia)^12.8502);
    mass_motor = mass_motor/1000;
end

function mass_esc = esc_weight(max_sus_current)
    mass_esc = 0.8013 * (max_sus_current^0.9727);
    mass_esc = mass_esc/1000;
end

% Run the main function
main_design();
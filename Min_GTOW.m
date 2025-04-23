% Define sweep parameters
mach = [0.2, 0.21, 0.22, 0.23, 0.24, 0.25];
thickness_chord_ratio = [0.12, 0.14, 0.16, 0.18];
num_of_blades = [4, 6, 8];
taper_ratio_wing = [0.3, 0.4, 0.5, 0.6];  
AR_rotor = [8, 9, 10];
DL_Rotor = [40, 50, 60, 70, 80, 90];
AR_wing = [8, 9, 10];
WL = [120, 130, 140, 150, 160, 170, 180];

% Fixed parameters
payload = 550;
Weight = 4 * payload;
RPM = 1800;
num_of_rotors = 8;
fuselage_length = 8;
fuselage_area = 6.76;
num_of_wheels = 4;
battery_capacity = 100000;
num_series_cells = 6;
max_current = 50;
motor_speed_constant = 1200;
max_rated_power = 10000;
max_sus_current = 40;

% Initialize results storage
results = [];
serial_no = 1;
min_gtow = inf;
min_gtow_params = [];

% Iterate over all combinations
for m = mach
    for tcr = thickness_chord_ratio
        for nb = num_of_blades
            for trw = taper_ratio_wing
                for ar = AR_rotor
                    for dl = DL_Rotor
                        for aw = AR_wing
                            for wl = WL
                                
                                % Derived parameters
                                cruise_speed = m * 330;
                                rotor_radius = sqrt(Weight / (pi * dl));
                                chord = rotor_radius / ar;
                                solidity = nb / (pi * ar);
                                wing_sweep = rad2deg(acos(0.75 / m));
                                wing_area = Weight / wl;
                                
                                % GTOW iteration
                                multiplier = 2;
                                step = 0.01;
                                tolerance = 5;
                                gtowf = payload * multiplier;
                                itr = 0;
                                
                                while true
                                    gtowi = payload * multiplier;
                                    itr = itr + 1;
                                    
                                    % Compute component weights
                                    hydraulics = hydraulics_weight(nb, chord, RPM, rotor_radius);
                                    instruments = instruments_weight(gtowf);
                                    controls = controls_weight(nb, chord, RPM, rotor_radius);
                                    wing = wing_weight(wing_area, aw, wing_sweep, trw, tcr, gtowf, cruise_speed);
                                    rotor = rotor_weight(nb, num_of_rotors, chord, rotor_radius, RPM);
                                    rotor_hub_hinge = rotor_hub_hinge_weight(nb, num_of_rotors, rotor_radius, RPM);
                                    fuselage = fuselage_weight(gtowf, fuselage_length, fuselage_area);
                                    landing_gear = landing_gear_weight(gtowf, num_of_wheels);
                                    battery = battery_weight(battery_capacity, num_series_cells);
                                    motor = motor_weight(max_current, motor_speed_constant, max_rated_power);
                                    esc = esc_weight(max_sus_current);
                                    
                                    gtowf = payload + hydraulics + instruments + controls + wing + rotor + rotor_hub_hinge + fuselage + landing_gear + battery + motor + esc;
                                    
                                    % Check convergence
                                    error = abs(gtowf - gtowi);
                                    if error < tolerance
                                        break;
                                    end
                                    
                                    % Adjust multiplier
                                    if gtowf < gtowi
                                        multiplier = multiplier + step;
                                    else
                                        step = step / 2;
                                        multiplier = multiplier + step;
                                    end
                                    
                                    if step < 1e-6
                                        break;
                                    end
                                end
                                
                                % Store results
                                results = [results; serial_no, m, tcr, nb, trw, ar, dl, aw, wl, gtowf];
                                
                                % Update minimum GTOW
                                if gtowf < min_gtow
                                    min_gtow = gtowf;
                                    min_gtow_params = [m, tcr, nb, trw, ar, dl, aw, wl];
                                end
                                
                                serial_no = serial_no + 1;
                            end
                        end
                    end
                end
            end
        end
    end
end

% Convert results to table
col_names = {'Serial_No', 'Mach', 'TCR', 'Num_Blades', 'Taper_Ratio', 'AR_Rotor', 'DL_Rotor', 'AR_Wing', 'WL', 'GTOW'};
results_table = array2table(results, 'VariableNames', col_names);

% Display results
disp(results_table);

% Display minimum GTOW result
fprintf('Minimum GTOW: %.2f kg\n', min_gtow);
fprintf('Corresponding Parameters: Mach=%.2f, TCR=%.2f, Num_Blades=%d, Taper_Ratio=%.2f, AR_Rotor=%d, DL_Rotor=%d, AR_Wing=%d, WL=%d\n', min_gtow_params);

% Function Definitions
function weight = hydraulics_weight(num_of_blades, blade_chord_length, RPM, rotor_radius)
    rotor_radius = rotor_radius * 3.281; % m to ft
    blade_chord = blade_chord_length * 3.281; % m to ft
    RPM = RPM * 2 * pi / 60; % rev/min to rad/s

    weight = 37 * (num_of_blades ^ 0.63) * (blade_chord ^ 1.3) * ((RPM * rotor_radius)/1000) ^ 2.1;
    weight = weight * 0.453592; % lb to kg
end

function weight = instruments_weight(gtow)
    gtow = gtow * 2.20462; % kg to lb
    weight = 3.5 * ((gtow / 1000) ^ 1.3);
    weight = weight * 0.453592; % lb to kg
end

function weight = controls_weight(num_of_blades, blade_chord, RPM, rotor_radius)
    rotor_radius = rotor_radius * 3.281; % m to ft
    blade_chord = blade_chord * 3.281; % m to ft
    RPM = RPM * 2 * pi / 60;

    weight = 36 * (num_of_blades) * (blade_chord ^ 2.2) * ((RPM * rotor_radius)/1000) ^ 3.2;
    weight = weight * 0.453592; % lb to kg
end

function weight = wing_weight(wing_area, ~, wing_sweep, taper_ratio_wing, thickness_chord_ratio, gtow, cruise_speed)
    ultimate_load_factor = 6;
    air_density = 1.006;

    dynamic_pressure = 0.5 * air_density * (cruise_speed ^ 2); % in Pa
    dynamic_pressure = dynamic_pressure * 47.88; % from Pa to lb/sq.ft
    wing_area = wing_area * 10.7639; % sq.m to sq.ft
    gtow = gtow * 2.20462; % kg to lb

    weight = (0.036 * (wing_area ^ 0.758) * ((wing_area / (cosd(wing_sweep) ^ 2)) ^ 0.6) * ...
             (dynamic_pressure ^ 0.006) * (taper_ratio_wing^ 0.04) * ...
             (((100 * thickness_chord_ratio) / cosd(wing_sweep)) ^ -0.3) * ...
             ((ultimate_load_factor * gtow) ^ 0.49));

    weight = weight * 0.453592; % lb to kg
end

function rotor_weight = rotor_weight(num_of_blades, num_of_rotors, blade_chord, rotor_radius, RPM)
    rotor_radius = rotor_radius * 3.281; % m to ft
    blade_chord = blade_chord * 3.281; % m to ft
    RPM = RPM * 2 * pi / 60;
    
    rotor_weight = 0.026 * (num_of_blades*0.66) * blade_chord * (rotor_radius ^ 1.3) * (RPM ^ 0.67) * (rotor_radius ^ 0.67);
    rotor_weight = rotor_weight * num_of_rotors;
    rotor_weight = rotor_weight * 0.453592; % lb to kg
end

function rotor_hub_hinge_weight = rotor_hub_hinge_weight(num_of_blades, num_of_rotors, rotor_radius, RPM)
    rotor_radius = rotor_radius * 3.281; % m to ft
    RPM = RPM * 2 * pi / 60;
    
    rotor_hub_hinge_weight = 0.037 * (num_of_blades * 0.28) * (rotor_radius ^ 1.5) * (RPM ^ 0.43) * (rotor_radius ^ 0.43);
    rotor_hub_hinge_weight = rotor_hub_hinge_weight * num_of_rotors;
    rotor_hub_hinge_weight = rotor_hub_hinge_weight * 0.453592; % lb to kg
end

function fuselage_weight = fuselage_weight(gtow, fuselage_length, fuselage_area)
    gtow = gtow * 2.20462; % kg to lb
    fuselage_length = fuselage_length * 3.281; % m to ft
    fuselage_area = fuselage_area * 10.7639; % m^2 to ft^2
    
    fuselage_weight = 6.9 * ((gtow / 1000) ^ 0.49) * (fuselage_length ^ 0.61) * (fuselage_area ^ 0.25);
    fuselage_weight = fuselage_weight * 0.453592; % lb to kg
end

function landing_gear_weight = landing_gear_weight(gtow, num_of_wheels)
    gtow = gtow * 2.20462; % kg to lb
    landing_gear_weight = (40 * ((gtow / 1000) ^ 0.67) * (num_of_wheels ^ 0.54)) * 1.1;
    landing_gear_weight = landing_gear_weight * 0.453592; % lb to kg
end

function mass_battery = battery_weight(battery_capacity, num_series_cells)
    mass_battery = 0.0418 * (battery_capacity ^ 0.9327) * (num_series_cells ^ 1.0725);
    mass_battery = mass_battery / 1000;
end

function mass_motor = motor_weight(~, motor_speed_constant, max_rated_power)
    mass_motor = 0.0109 * (motor_speed_constant ^ 0.5122) * (max_rated_power ^ -0.1902);
    mass_motor = mass_motor / 1000;
end

function mass_esc = esc_weight(max_sus_current)
    mass_esc = 0.8013 * (max_sus_current ^ 0.9727);
    mass_esc = mass_esc / 1000;
end

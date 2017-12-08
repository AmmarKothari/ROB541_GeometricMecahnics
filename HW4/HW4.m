%% Plotting Connection Field for front wheel ackerman steering 

A = @(alpha1, alpha2) -[1, 0;
                        0, 0;
                        tan(alpha2), 0];
step = 0.1;
lower = -1;
upper = 1;
alpha1_range = lower:step:upper;
alpha2_range = lower:step:upper;
A_all = cell(length(alpha1_range), length(alpha2_range));
for a1 = 1:numel(alpha1_range)
    for a2 = 1:numel(alpha2_range)
        A_all{a1, a2} =  A(alpha1_range(a1), alpha2_range(a2));
    end
end

PlotConnection(A_all, alpha1_range, alpha2_range)

%% Plotting Connection Field for rear wheel ackerman steering
syms R alpha2 l
% w = [1, 0, 0, -R, 0;
%     cos(alpha2) sin(alpha2), l*sin(alpha2), -R, 0;
%     0, 1, 0, 0, 0;
%     -sin(alpha2), cos(alpha2), l*cos(alpha2), 0, 0;];
w = [1, 0, 0, -R, 0;
    0, 1, 0, 0, 0;
    -sin(alpha2), cos(alpha2), l*cos(alpha2), 0, 0;];
wg = w(:, 1:3);
wb = w(:, 4:5);
A = inv(wg) * wb;
A = subs(A, [R l], [1, 1]);
A_func = @(a1, a2) subs(A, alpha2, a2);

step = 0.1;
lower = -1;
upper = 1;
alpha1_range = lower:step:upper;
alpha2_range = lower:step:upper;
A_all = cell(length(alpha1_range), length(alpha2_range));
for a1 = 1:numel(alpha1_range)
    for a2 = 1:numel(alpha2_range)
        A_all{a1, a2} =  A_func(alpha1_range(a1), alpha2_range(a2));
    end
end

PlotConnection(A_all, alpha1_range, alpha2_range)
suptitle('Local Connection Vector Field for Rear Wheel Ackerman Steering')
% saveas(gcf, 'AckermanRearWheel', 'png');

%% Plotting Connection Field for front wheel ackerman steering
syms R alpha2 l
% w = [1, 0, 0, -R, 0;
%     cos(alpha2) sin(alpha2), l*sin(alpha2), -R, 0;
%     0, 1, 0, 0, 0;
%     -sin(alpha2), cos(alpha2), l*cos(alpha2), 0, 0;];
w = [cos(alpha2), sin(alpha2), l*sin(alpha2), -R, 0;
    0, 1, 0, 0, 0;
    -sin(alpha2), cos(alpha2), l*cos(alpha2), 0, 0;];
wg = w(:, 1:3);
wb = w(:, 4:5);
A = inv(wg) * wb;
A = subs(A, [R l], [1, 1]);
A_func = @(a1, a2) subs(A, alpha2, a2);

step = 0.1;
lower = -1;
upper = 1;
alpha1_range = lower:step:upper;
alpha2_range = lower:step:upper;
A_all = cell(length(alpha1_range), length(alpha2_range));
for a1 = 1:numel(alpha1_range)
    for a2 = 1:numel(alpha2_range)
        A_all{a1, a2} =  A_func(alpha1_range(a1), alpha2_range(a2));
    end
end

PlotConnection(A_all, alpha1_range, alpha2_range)
suptitle('Local Connection Vector Field for Front Wheel Ackerman Steering')

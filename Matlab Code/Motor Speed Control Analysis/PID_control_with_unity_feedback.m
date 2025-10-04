clear all;
format short;

% Feedback Gain 
k = 1;

% PID Coefficients
kd = 1;
kp = 100;
ki = 1;

% PID Controller
PID = tf([kd kp ki], [1 0]);

% Motor (Plant)
Motor = tf([1], [1 2]);

% Feedforward Control
H = tf([1 2], [1])

% Feedback System
CLTF = feedback((PID + H) * Motor, k);

% Poles
system_poles = pole(CLTF);
instable_poles = [];

% Stability Checker
zero_detected = false;
instable = false;
for i = 1:size(system_poles, 1)
    if (real(system_poles(i)) == 0)
        if (not(zero_detected))
            zero_detected = true;
        else
            instable = true;
            instable_poles(i) = system_poles(i);
        end
    end
    if (real(system_poles(i)) > 0)
        instable = true;
        instable_poles(i) = system_poles(i);
    end
end

disp("Your system has the following closed loop transfer function: ")
CLTF
disp("and the open loop transfer function:")
PID * Motor

if instable
    disp("The system is unstable. The instable poles are:")
    disp(instable_poles')
    disp("No further tests will be performed.")
    quit;
else
    disp("The system is stable. Moving onto performance tests.")
end

% Peformance Test
t = 0:0.01:20;
target = 5;
input = inputFunction(t);
lsimplot(CLTF, input, t, "g")

%hold on;
%plot(t, 1.05*target*ones(size(t)), "--");
%hold on;
%plot(t, 0.95*target*ones(size(t)), "--");
%hold on;

%ylim([-1 target*1.1]);
ylabel("Speed (m/s)")
%legend("Response", "Upperbound", "Lowerbound")
legend("Response")

function val = inputFunction(t)
    for i = 1:size(t, 2)
        val(i) = exp(-t(i)^2) + 2*exp(-0.5*(t(i)-3)^2) + 4*exp(-2*(t(i)-5)^2) + 2*exp(-0.25*(t(i)-7)^2) + 3*exp(-(t(i)-9)^2);
    end
end
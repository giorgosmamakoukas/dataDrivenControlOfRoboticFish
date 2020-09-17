function [Psi] =psivec(s)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
  %Declare states from the first six observables
	 x = s(2);  y = s(3);  psi = s(4);
	 v1 = s(5);  v2 = s(6); omega = s(7);
     u=[s(8) s(9)]; 

if v2 == 0 && v1 == 0
   atanv1v2 = 0; % 0/0 gives NaN
   psi38 = 0;
   psi41 = 0;
   psi54 = 0;
   psi58 = 0;
   psi65 = 0;
   psi67 = 0;
   psi68 = 0;
   psi72 = 0;
   psi74 = 0;
else
   atanv1v2 = atan(v2/v1);
   psi38 = v1 * v2^2 * omega/sqrt(v1^2+v2^2);
   psi41 = v1^2 * v2 * omega / sqrt(v1^2 + v2^2) * atanv1v2;
   psi54 = v1^2 * v2 * omega / sqrt(v1^2 + v2^2);
   psi58 = v1 * v2^2 * omega * atanv1v2 / sqrt(v1^2 + v2^2);
   psi65 = u(2) * v1 * v2 / sqrt(v1^2 + v2^2);
   psi67 = u(1) * v1 * v2 * atanv1v2 / sqrt(v1^2 + v2^2);
   psi68 = u(2) * v2^2 * atanv1v2 / sqrt(v1^2 + v2^2);
   psi72 = u(1) * v1 * v2 / sqrt(v1^2 + v2^2);
   psi74 = u(2) * v1 * v2 * atanv1v2 / sqrt(v1^2 + v2^2);
end

% States
Psi(1) = x;
Psi(2) = y;
Psi(3) = psi;
Psi(4) = v1;
Psi(5) = v2;
Psi(6) = omega;

% f(t)
Psi(7)  = v1 * cos(psi) - v2 * sin(psi);
Psi(8)  = v1 * sin(psi) + v2 * cos(psi);
Psi(9) =  v2 * omega;
Psi(10) = v1^2;
Psi(11) = v2^2;
Psi(12) = v1 * omega;
Psi(13) = v1 * v2;
Psi(14) = sign(omega) * omega^2;

% df(t)/dt
Psi(15) = v2 * omega * cos(psi);
Psi(16) = v1^2 * cos(psi);
Psi(17) = v2^2 * cos(psi);
Psi(18) = v1 * omega * sin(psi);
Psi(19) = v1 * v2 * sin(psi);

Psi(20) = v2 * omega * sin(psi);
Psi(21) = v1^2 * sin(psi);
Psi(22) = v2^2 * sin(psi);
Psi(23) = v1 * omega * cos(psi);
Psi(24) = v1 * v2 * cos(psi);  

Psi(25) = v1 * omega^2;
Psi(26) = v1 * v2 * omega;
Psi(27) = v1 * v2^2;
Psi(28) = v2 * sign(omega) * omega^2;
Psi(29) = v1^3;

%d/dt of dot{v2}
Psi(30) = v2 * omega^2;
Psi(31) = v1 * omega * sqrt(v1^2 + v2^2);
Psi(32) = v2 * omega * sqrt(v1^2 + v2^2) * atanv1v2;
Psi(33) = v1^2 * v2;
Psi(34) = v1 * sign(omega) * omega^2;
Psi(35) = v2^3;
Psi(36) = v1^3 * atanv1v2;
Psi(37) = v1 * v2^2 * atanv1v2;
Psi(38) = psi38;
Psi(39) = v1^2 * v2 * atanv1v2^2;
Psi(40) = v2^3 * atanv1v2^2;
Psi(41) = psi41;
% Psi(42) = v1^3 * omega / sqrt(v1^2 + v2^2);

% d/dt of dot{omega}
Psi(42) = v2^2 * omega;
Psi(43) = v1 * v2 * sqrt(v1^2 + v2^2);
Psi(44) = v2^2 * sqrt(v1^2 + v2^2) * atanv1v2;
Psi(45) = v1^2 * omega;
Psi(46) = v1^2 * sqrt(v1^2 + v2^2) * atanv1v2;
Psi(47) = v1 * v2 * sign(omega) * omega;
Psi(48) = omega^3;

%d/dt of dot{v1}
Psi(49) = v2 * omega * sqrt(v1^2 + v2^2); % 33
Psi(50) = v1^3; % 37
Psi(51) = v1 * v2^2; % 35
Psi(52) = v1^2 * v2 * atanv1v2; % 39
% v2^3 * atanv1v2 in 60
Psi(53) = psi54; % 40

Psi(54) = v1 * omega * sqrt(v1^2 + v2^2) * atanv1v2; % 34
Psi(55) = v1^3 * atanv1v2^2; % 42
Psi(56) = v1 * v2^2 * atanv1v2^2; % 41
Psi(57) = psi58; % 43
Psi(58) = v2^3 * atanv1v2;  % 38
% Psi(60) = v2^3 * omega / sqrt(v1^2 + v2^2); %%%%%%%%%%%%%%

Psi(59) = v1 * omega^2; % 32
Psi(60) = v2 * sign(omega) * omega^2; % 36

% add control inputs
Psi(61) = u(1);  
Psi(62) = u(2);

Psi = Psi';

end


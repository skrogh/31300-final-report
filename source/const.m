addpath('subsystems');

A1 = 28;
A2 = 28;
a1 = 0.071;
a2 = 0.071;
k1 = 3.14;
k2 = 3.29;
km = 0.50;
g = 981;

h10 = 12.6;

% Opg 1
% model i ulin_mod

% Opg 2
% Systemet:
% A1*dh1/dt = -a1*sqrt( 2*g*h1 ) + a2*sqrt( 2*g*h2 ) + k1*d
% A2*dh2/dt = -a2*sqrt( 2*g*h2 ) + k2*u
% dh1/dt = 0, dh2/dt = 0
% h1 = h10, h2 = h20, d = 0, u = u0
% Løses for h20 og u0.

u0 = a1*sqrt(2)*sqrt(g*h10)/k2;
h20 = a1^2*h10/a2^2;

dh20 = 1/2 * sqrt(2) * g / sqrt(g*h20);
dh10 = 1/2 * sqrt(2) * g / sqrt(g*h10);

% Opg 3
% se sim.

% Opg 4
% Systemet er givet ved:
% A1*dh1/dt = -a1*sqrt(2*g*h1) + a2*sqrt(2*g*h2) + k1*d
% A2*dh2/dt = -a2*sqrt(2*g*h2) + k2*u
% lineariseret ved:
% A1*s*h1 = -a1*dh10*h1 + a2*dh20*h2 + k1*d
% A2*s*h2 = -a2*dh20*h2 + k2*u

% h1/u = a2*dh20*k2/(A1*A2*s^2+A1*a2*s*dh20+A2*a1*s*dh10+a1*a2*dh10*dh20)
% h2/u = k2/(A2*s+a2*dh20)
% h1/d = k1/(A1*s+a1*dh10)
% h2/d = 0
h1u = [ 0,0, a2*dh20*k2 ; A1*A2, A1*a2*dh20 + A2*a1*dh10, a1*a2*dh10*dh20 ];
h2u = [ 0, k2; A2, a2*dh20 ];
h1d = [ 0, k1; A1, a1*dh10 ];
h2d = [ 0; 1 ];

h1u = tf( h1u(1,:), h1u(2,:) );
h2u = tf( h2u(1,:), h2u(2,:) );
h1d = tf( h1d(1,:), h1d(2,:) );
h2d = tf( h2d(1,:), h2d(2,:) );

fig_step = 1;
fig_step = figure(fig_step);

% hvodan sætter man interne begyndelses betingelser / lægger offset til?
opt_u = stepDataOptions( 'InputOffset', 0, 'StepAmplitude', 0.1 );
opt_d = stepDataOptions( 'InputOffset', 0, 'StepAmplitude', 0.1 );
subplot( 2, 2, 1 ), plot( step( h1u, opt_u ) + h10 );
subplot( 2, 2, 2 ), plot( step( h2u, opt_u ) + h20 );
subplot( 2, 2, 3 ), plot( step( h1d, opt_d ) + h10 );
subplot( 2, 2, 4 ), plot( step( h2d, opt_d ) + h20 );
% anyway... det passer med simulink

% Opg 6
fig_pz = 2;
fig_pz = figure(fig_pz);
subplot( 2, 2, 1 ), pzmap( h1u );
subplot( 2, 2, 2 ), pzmap( h2u );
subplot( 2, 2, 3 ), pzmap( h1d );
subplot( 2, 2, 4 ), pzmap( h2d );

fig_pz = 3;
fig_pz = figure(fig_pz);
subplot( 2, 2, 1 ), bode( h1u );
subplot( 2, 2, 2 ), bode( h2u );
subplot( 2, 2, 3 ), bode( h1d );
subplot( 2, 2, 4 ), bode( h2d );

figure(5)
reg1


regulator = Kp*tf( [tau_i 1], [tau_i 0] )*tf( [tau_d 1], [alpha*tau_d 1] );
[ reg_num, reg_den ] = tfdata( regulator );


regulator_h = tf( [tau_d 1], [alpha*tau_d 1] );
regulator_g = Kp*tf( [tau_i 1], [tau_i 0] );
[ reg_num, reg_den ] = tfdata( regulator );
[ reg_num_h, reg_den_h ] = tfdata( regulator_h );
[ reg_num_g, reg_den_g ] = tfdata( regulator_g );



%#ok<*NBRAK>
% remove warning for tf([1],[....])

G1 = 1; 
G2 = h1u;
H = km;
Gc = @(Kp, tau_i, tau_d, alpha) Kp*tf( [tau_i 1], [tau_i 0] )*tf( [tau_d 1], [alpha*tau_d 1] );

%open and closed sys is from r -> c_m,
%closed_sys_r_c is closed loop from r -> c
open_sys = @(Kp, tau_i, tau_d, alpha) Gc(Kp, tau_i, tau_d, alpha)*G1*G2*H;
closed_sys = @(Kp, tau_i, tau_d, alpha) feedback( open_sys(Kp, tau_i, tau_d, alpha), 1 );
closed_sys_r_c = @(Kp, tau_i, tau_d, alpha) feedback( open_sys(Kp, tau_i, tau_d, alpha)/H, H );


%resolution for magnitude/phase search
resolution = 0.001;

%Make initial, naive controller:
%Inital phasemargin aim:
gamma_m = 50; % fasemargen mål
alpha = 0.1
%calc phi_m
[mag, pha] = bode( tf( [1 1], [alpha 1]),  10.^(-5:resolution:5) );
phi_m = max(pha) % ekstra fasemargen fra lead-led
phi_i = -11; % fasemargen reserveret til I-led
search_phase = -180 + gamma_m - phi_m - phi_i

%Generate phase plot data
[mag, pha, Wout] = bode( G1*G2*H, 10.^(-5:resolution:5) );
mag = squeeze(mag);
pha = squeeze(pha);
Wout = squeeze(Wout);
%calculate distance from search phase
temp = abs( pha - search_phase );
%find index of closest value (index of searchphase)
[temp, index] = min( temp );
%magnutude at search phase
mag_at = mag(index);
%frequency of search phase
W_at = Wout(index)
pha_at = pha(index)

%set initial Kp
initial_Kp = 1/mag_at;
W_c = W_at;

%plot for visual check
figure(1)
bode( initial_Kp*G1*G2*H );
margin( initial_Kp*G1*G2*H );


%find tau_d
tau_d = 1/(W_c * sqrt(alpha));
figure(2)
bode( tf( [tau_d 1], [tau_d*alpha, 1] ) );

%find tau_i that gives phi_i at W_c
[mag, pha, Wout] = bode( tf( [1 1], [1 0] ), 10.^(-5:resolution:5) );
mag = squeeze(mag);
pha = squeeze(pha);
Wout = squeeze(Wout);
temp = abs( pha - phi_i );
[temp, index] = min( temp );
mag_at = mag(index);
W_at = Wout(index);
N = W_at;
tau_i = N/W_c;

figure(3)
bode( tf( [tau_i 1], [tau_i 0] ) )

%plot for visual check
figure(4)
bode( open_sys( initial_Kp, tau_i, tau_d, alpha ) );

%find Kp that gives a gain of 1 at W_c
temp = bode( open_sys(1, tau_i, tau_d, alpha), W_c );
Kp = 1/(temp);

figure(5)
bode( open_sys( Kp, tau_i, tau_d, alpha ), 10.^(-5:resolution:6) )
margin( open_sys( Kp, tau_i, tau_d, alpha ) )
allmargin( open_sys( Kp, tau_i, tau_d, alpha ) )

figure(6)
step( closed_sys( Kp, tau_i, tau_d, alpha ) )

figure(7)
bode( closed_sys( Kp, tau_i, tau_d, alpha ) )


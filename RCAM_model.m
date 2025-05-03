% RCAM mathematical model 
function [xdot] = RCAM_model(X,U)

% defining state and control vectors 
x1 = X(1) ;     %u
x2 = X(2) ;     %v
x3 = X(3) ;     %w
x4 = X(4) ;     %p
x5 = X(5) ;     %q
x6 = X(6) ;     %r
x7 = X(7) ;     %phi
x8 = X(8) ;     %theta
x9 = X(9) ;     %psi


u1 = U(1) ;     % d_A aileron
u2 = U(2) ;     %d_T stabilizer 
u3 = U(3) ;     % d_R rudder 
u4 = U(4) ;     % d_th1 throttle 1
u5 = U(5) ;     %d_th2 throttle 2

% ------------------constants----------------------
% mass of vehical 
m = 120000; % asummed to be constant in kg 

cbar = 6.6;     % mean aerodynamic chord in m 
lt = 24.8;      % distance from AC of tail to CG of the Aircraft 
s = 260;        % wing planform area in (m^2)  
st = 64;        % tail planform Area in (m^2) 

% podition of CG of the Aircraft 
Xcg = 0.23*cbar;
Ycg = 0;
Zcg = 0.10*cbar;

% position of aerodynamic center of the Aircraft 
Xac = 0.12*cbar;
Yac = 0;
Zac = 0;

% engine constants 
% engine 1 
Xapt1 = 0;
Yapt1 = -7.94;
Zapt1 = -1.9;

%engine 2 
Xapt2 = 0;
Yapt2 = 7.94;
Zapt2 = -1.9;

% other contants 
rho = 1.225;
g = 9.81;
depsda = 0.25; % change in downwash with change in AOA 
alpha_L0 = -11.5*pi/180;
n = 5.5;            % slope of lift curve slope 
a3 = -768.5;        % coefficient of alpha^3
a2 = 609.2;         % coefficient of alpha^2
a1 = -155.2;        % coefficient of alpha^1
a0 = 15.212;        % coefficient of alpha^0
alpha_switch = 14.5*pi/180;  % alpha where lift curve slope change from linear to non linear 


%% -------------control limits/saturations-----------------------

% u1min = -25*pi/180;
% u1max =  25*pi/180;
% 
% u2min = -25*pi/180;
% u2max = 10*pi/180;
% 
% u3min = -30*pi/180;
% u3max = 30*pi/180;
% 
% u4min = 0.5*pi/180;
% u4max = 10*pi/180;
% 
% u5min = 0.5*pi/180;
% u5max = 10*pi/180;
% 
% if (u1>u1max)
%     u1 = u1max;
% elseif (u1<u1min)
%     u1 = u1min;
% end 
% 
% if (u2>u2max)
%     u2 = u2max;
% elseif (u2<u2min)
%     u2 = u2min;
% end 
% 
% if (u3>u3max)
%     u3 = u3max;
% elseif (u3<u3min)
%     u3 = u3min;
% end 
% 
% if (u4>u4max)
%     u4= u4max;
% elseif (u4<u4min)
%     u4 = u4min;
% end 
% 
% if (u5>u5max)
%     u5 = u5max;
% elseif (u5<u5min)
%     u5 = u5min;
% end 
% 

%% -----------------variables------------------
% calculate airspeed 
Va = sqrt(x1^2 + x2^2 + x3^2);

% calculate alph and beta 
alpha = atan2(x3,x1);
beta = asin(x2/Va);

% calculate dynamic pressure 
Q = 0.5*rho*Va^2 ;

% also define vector wbe_b and v_b 
wbe_b = [x4;x5;x6];
v_b = [x1;x2;x3];

% --------------------------aerodynamic force coefficients------------------
% calculate cL_wb (wing body combination) 
if alpha < alpha_switch
    cL_wb = n*(alpha - alpha_L0);
else 
    cL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha^1 + a0 ;
end 


% calculate cL 
epsilon = depsda * (alpha - alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;
cL_t = 3.1*(st/s)*alpha_t;

% total lift force 
cL = cL_wb + cL_t;

% total drag 
cD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;

% calculate side force 
cY = -1.6*beta + 0.24*u3;


% ---------------dimensional aerodynamic forces---------------
FA_s = [-cD*Q*s;
        cY*Q*s;
        -cL*Q*s];

% rotate these forces to F_b body axis 
c_bs = [cos(alpha) 0 -sin(alpha) ;
    0 1 0 ;
    sin(alpha) 0 cos(alpha) ];
FA_b = c_bs * FA_s ;

%---------dimensional moment coefficients about AC --------------
eta11 = -1.4*beta;
eta21 = -0.59 -(3.1*(st * lt)/(s*cbar))*(alpha - epsilon);
eta31 = (1 - alpha*(180/(15*pi)))*beta;

eta = [eta11;
    eta21;
    eta31];

dCMdx =(cbar/Va)*  [-11 0 5;
    0 (-4.03*(st*lt^2)/(s*cbar^2)) 0 ;
    1.7 0 -11.5];

dCMdu = [-0.6 0 0.22;
    0 (-3.1*(st*lt)/(s*cbar)) 0 ;
    0 0 -0.63];

% calculate cM 
cMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];

% --------aerodynamic moment about AC-------------------

Mac_b = cMac_b * Q * s * cbar;

% --------aerodynamic moment about cg-------
rcg_b = [Xcg ; Ycg ; Zcg];
rac_b = [Xac ; Yac ; Zac];

MAcg_b = Mac_b + cross(FA_b , rcg_b - rac_b);


%-------------engine force and moments-----------------

% calculate thrust of each engine 
F1 = u4*m*g;
F2 = u5*m*g;

% asumme engine thrust alligned with body axis 
FE1_b = [F1;0;0];
FE2_b = [F2;0;0];

FE_b = FE1_b + FE2_b ;

% moment due to engine thrust due to offset from cog of A/c
mew1 = [Xcg - Xapt1;
    Yapt1 - Ycg;
    Zcg - Zapt1];

mew2 = [Xcg - Xapt2;
    Yapt2 - Ycg;
    Zcg - Zapt2];

MEcg1_b = cross(mew1,FE1_b);
MEcg2_b = cross(mew2 ,FE2_b);

MEcg_b = MEcg1_b + MEcg2_b;


%------gravity effects-----------------
%--garvity from earth axis to body axis 
g_b = [-g*sin(x8);
    g*cos(x8)* sin(x7);
    g*cos(x8)*cos(x7)];

Fg_b = m*g_b;


% ------------state derivative-------------
ib = m* [40.07 0 -2.0923;
     0 64 0 ;
    -2.0923 0 99.92];

invib = inv(ib) ;

F_b = Fg_b + FE_b + FA_b;

x1to3dot = (1/m)*F_b - cross(wbe_b , v_b);

% compute pdot, qdot, rdot

Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invib *(Mcg_b - cross(wbe_b , ib*wbe_b));


% compute phidot, thetadot , psidot 

H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7) -sin(x7) 
         0 sin(x7)*sec(x8) cos(x7)*sec(x8)];

x7to9dot = H_phi * wbe_b ;

% place in first order from 

%% defining the rotation matrix for the navigation equations 
C1v = [cos(x9) -sin(x9) 0 ;
    sin(x9) cos(x9) 0 ;
    0 0 1];


C21 = [ cos(x8) 0 -sin(x8) ;
    0 1 0 ;
    sin(x8) 0 cos(x8)];

Cb2 = [ 1 0 0 ;
    0 cos(x7) sin(x7);
    0 -sin(x7) cos(x7)];

Cbv = C1v*C21*Cb2;

Cvb = Cbv';

x10to12 = Cvb * v_b;

xdot = [x1to3dot;
        x4to6dot;
        x7to9dot;
        x10to12];





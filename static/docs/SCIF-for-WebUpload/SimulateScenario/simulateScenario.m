
clear all;
clc;
close all;

global T ;

nx = 5;     %state-vector dim.
nz = 2;     %mst vector dim.

%%%============= Parameters setting ======================

nExpt = 100;                   % number of MC runs


nSensorArray = [2:1:15];

T = 2; % sampling rate

%quatities associate. state eqn
v1_sigma = 5e-2;     %std. dev associated with linear part of process eqn.
v2_sigma = 1e-3;      %std dev associated with turning motion

%%% 2,6
G   = [  T^2/2   0       0;                
         T       0       0;
         0       T^2/2  0;
         0       T       0;
         0       0  T];

omgArray   = ((2*pi)/360)*[5 -9 -3 9]; %% crucial turn rate (Omega/omg) points

k_seg = [20 15 10 5]; %% crucial time segments (sampled)

N = sum(k_seg);

xArray = zeros(nx,N,nExpt);

S0 = diag([10 5 10 5 0.1*(2*pi)/360]);

x0 =  [0 100 -400 -120 (2*pi)/360*5]';

x0Array = repmat(x0,[1,nExpt]) + S0*randn(nx,nExpt); 

for expt = 1:nExpt
    
    
    x  = x0;         %% initial state [x_pos, x_vel, y_pos, y_pos]
    
    cum_kSeg = cumsum(k_seg,2);
    
    for k = 1 : N,
        
        if k <= cum_kSeg(1),
            omg = omgArray(1);
        end
        if k > cum_kSeg(1) && k <= cum_kSeg(2),
            omg = omgArray(2);
        end
        if k > cum_kSeg(2) && k <= cum_kSeg(3),
            omg = omgArray(3);
        end
        if k > cum_kSeg(3) && k <= cum_kSeg(4),
            omg = omgArray(4);
        end
        
        x(5) = omg;
        
        x = StateEq(x) + G*[v1_sigma*randn(2,1); v2_sigma*randn];
        
        xArray(:,k,expt) = x;
        
    end %time k
    
end % MC run


sCell = cell(1,length(nSensorArray));

for ns  = 1:length(nSensorArray)
    
    nSensors = nSensorArray(ns);
    
    sArray = zeros(2,nSensors,nExpt);
    
    for expt = 1:nExpt
        
        sArray(:,:,expt) = 4e3*(2*rand(2,nSensors)-1);     %sensor pos.
        
    end;
    
    sCell{1,ns} = sArray;
end

R = diag([100 100]);

Rsqrt = sqrt(R);

R_inv = pinv(R);

save C:\Haran\Research\InfoFilters\CT\SCIF-CIF-EKF\tstInfo;

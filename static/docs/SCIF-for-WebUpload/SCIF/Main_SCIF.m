
%%%%%%%%%=================================================================
%%%%%%%%% This m-code implements SCIF (Square-root Cubature Information Filter)
%%%%%%%%% Programmer: I. Arasaratnam
%%%%%%%%% Note: To modify the simaultion scenarion, you will have to modify
%%%%%%%%% simulateScenario.m which can be found in simulateScenario folder
%%%%%%%%%=================================================================


clear all;
clc;
close all;

global Qsqrt Rsqrt Rsqrt_inv QPtArray wPtArray nPts T nx nz;

nx = 5;     %state-vector dim.
nz = 2;     %mst vector dim.

[QPtArray,wPtArray,nPts] = findSigmaPts(nx);

load C:\Haran\Research\InfoFilters\CT\SCIF-CIF-EKF\tstInfo;

q1_sig2 = 1e-2;     %std. dev associated with linear part in process eqn.
q2_sig2 = 1e-3;      %std dev ass. with turning motion

Qsqrt = G*sqrt(diag([q1_sig2 ; q1_sig2 ;q2_sig2]));

Rsqrt_inv = pinv(Rsqrt);

devCount = zeros(1,length(nSensorArray));

for ns  = 1:length(nSensorArray)
    
    MSEpos = zeros(1,N);
    MSEvel = zeros(1,N);
    MSEomg = zeros(1,N);
    
    nSensors = nSensorArray(ns);    
    
    fprintf('No of sensors in process = %d\n',nSensors);
    
    xestArray = zeros(nx,N,nExpt);
    
    for expt = 1:nExpt
        
        s = sCell{1,ns}(:,:,expt);     %sensor pos.    
        
        Skk = S0;
        xkk = x0Array(:,expt);
     
        Sykk = pinv(Skk);
        ykk = Sykk*Sykk'*xkk;
        
        ykkArray = repmat(ykk,[1,nSensors]);
        SykkArray = repmat(Sykk,[1,1,nSensors]);
        
        %%%initilaization for global filter
        xkk_gl = xkk;
        Skk_gl = Skk;        
        
        ykk_gl = ykk;
        Sykk_gl = Sykk;
        
        xestArray = [];
        
        for k = 1 : N,

            [xkk1_gl,Skk1_gl] = Predict(xkk_gl,Skk_gl);
            
            [ykk1_gl,Sykk1_gl] = Convert_state_2_info(xkk1_gl,Skk1_gl);            
         
            iArray = zeros(nx,nSensors);
            
            SIArray = zeros(nx,nSensors*nz);            
            
            x = xArray(:,k,expt);
         
            for j = 1:nSensors
                
                z = MstEq(x,s(:,j)) + Rsqrt*randn(2,1);                
          
                [ik,Ik] =Update(ykk1_gl,Sykk1_gl,xkk1_gl,Skk1_gl,z,s(:,j));
                
                iArray(:,j) = ik;
                
                SIArray(:,(j-1)*nz+1:j*nz) = Ik;
                
            end;
            
            %%%============================================================
            %%%  Info Fusion: global filter
            %%%============================================================
            
            [ykk_gl,Sykk_gl] = fuseInfo(iArray, SIArray,ykk1_gl,Sykk1_gl);      
            
            [xkk_gl,Skk_gl] = Convert_info_2_state(ykk_gl,Sykk_gl);
            
             %%%============================================================
            
            xestArray(:,k,expt) = xkk_gl;
            
            delta_pos(k) = sum( (x([1 3])-xkk_gl([1 3])).^2); % deviation in pos
            delta_vel(k) = sum( (x([2 4])-xkk_gl([2 4])).^2);
            delta_omg(k) = (x(5)-xkk_gl(5))^2;            
        
        end
        
        %%%===============================================================
        %%%%  detect any filter deviation?
        %%%================================================================
        
        rmserr = sqrt(mean(delta_pos));  % calculate root-mean-squared error(rmserr)
        if   rmserr > 1e2
            
            devCount(ns) = devCount(ns)+1;
            
        else
            
            MSEpos = MSEpos + delta_pos;
            MSEvel = MSEvel + delta_vel;
            MSEomg = MSEomg + delta_omg;           
            
        end;        

    end % MC run
    
    nExpt_new = nExpt - devCount(ns);
    
    
    RMSEpos = sqrt(MSEpos/nExpt_new);
    RMSEvel = sqrt(MSEvel/nExpt_new);
    RMSEomg = sqrt(MSEomg/nExpt_new);
    
    
    BigRMSEpos(ns) = sqrt(sum(MSEpos)/(nExpt_new*N));
    BigRMSEvel(ns) = sqrt(sum(MSEvel)/(nExpt_new*N));
    BigRMSEomg(ns) = sqrt(sum(MSEomg)/(nExpt_new*N));
    
    
end % nSensors

RMSEpos_scif = BigRMSEpos;
RMSEvel_scif = BigRMSEvel;
devCount_scif = devCount;

%%%==== Plotting ==========================================================
close all;
figure;
plot(xArray(1,:,end),xArray(3,:,end),'r','linewidth', 2);
hold on;
plot(xestArray(1,:,end),xestArray(3,:,end),'k-.','linewidth', 3);
plot(s(1,:),s(2,:),'pr','markerfacecolor','r');

ylabel('Y','fontsize',12,'fontweight','Bold');
xlabel('X','fontsize',12,'fontweight','Bold');
grid on;
legend('True','Estimated','Sensor', 2);
% axis tight;




time = [T:T:T*cum_kSeg(end)];

close all;
figure;
semilogy(nSensorArray,BigRMSEpos, 'r','linewidth', 3);
grid on;
ylabel('ARMSE [Pos] (m)','fontsize',12,'fontweight','Bold');
xlabel('Number of Sensors, n_{s}','fontsize',12,'fontweight','Bold');
% axis([1 nSensors 70 2250]);
set(gca,'XTick',1:2:nSensors);

figure;
semilogy(nSensorArray,BigRMSEvel, 'r','linewidth', 3);
grid on;
ylabel('ARMSE [Vel] (m/s)','fontsize',12,'fontweight','Bold');
xlabel('Number of Sensors, n_{s}','fontsize',12,'fontweight','Bold');
% axis([1 nSensors 10 250]);
set(gca,'XTick',1:2:nSensors);



figure;
plot(nSensorArray,BigRMSEomg);
xlabel('No. of Sensors');
ylabel('ARMSE [Angular Vel]');


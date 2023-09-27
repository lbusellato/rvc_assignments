%--------------------------------------------------------------------------
%
%   generateTrapezoidal.m
%
%   This script implements the computation of trapezoidal velocity
%   profiles, with possible constraints being tc, dqc, ddqc, both dqc and
%   ddqc, dqi and dqf as well as ddq max,  dqi and dqf as well as ddq max
%   and dq max. The trajectory is computed only if it is feasible given the
%   constraints.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [q,v,a,t] = generateTrapezoidal(ti, tf, qi, qf, tc, vc, ac, dqi, dqf, am, vm)
    sgn = 1; argCheck = false; msg = ".";
    % Input args check
    if qf == qi
        % Not much to do here...
        ta = 0; td = 0; tc = 0; dqc = 0; ddqc = 0; dqi = 0; dqf = 0;
    elseif ~isnan(vc) && ~isnan(ac) % Both dqc and ddqc fixed
        if ac ~= 0 && vc ~= 0
            ac = sign(qf-qi)*ac; vc = sign(qf-qi)*vc;  % Handle qf < qi case
            tc = vc / ac;
            % In this case tf is computed and not given
            tf = (vc^2+ac*(qf-qi))/(vc*ac) + ti;
            dqc = vc; ddqc = ac; ta = tc; td = tc; dqi = 0; dqf = 0;
        else
            argCheck = true; msg = ": ddqc and dqc must be different than zero.";
        end
    elseif ~isnan(tc) % tc fixed
        if tc <= (tf-ti) / 2
            ddqc = (qf-qi)/(tc*(tf-ti)-tc*tc);
            dqc = ddqc*tc; ta = tc; td = tc; dqi = 0; dqf = 0;
        else
            argCheck = true; msg = ": tc must be less than (tf-ti)/2.";
        end
    elseif ~isnan(vc) % dqc fixed
        vc = sign(qf-qi)*vc; % Handle qf < qi case
        tc = ((qi-qf)+vc*(tf-ti))/vc;
        if tc > 0 && tc <= (tf-ti)
            ddqc = vc / tc;
            dqc = vc; ta = tc; td = tc; dqi = 0; dqf = 0;
        else
            argCheck = true; msg = ": tc must be less than (tf-ti)/2.";
        end
    elseif ~isnan(ac) % ddqc fixed
        ac = sign(qf-qi)*ac;  % Handle qf < qi case
        f = (ac*(tf-ti)^2-4*(qf-qi))/ac;
        if f >= 0 % Check the argument of sqrt
            tc = 0.5*(tf-ti) - 0.5 * sqrt(f);
            if tc <= (tf-ti) / 2
                dqc = ac * tc;
                ddqc = ac; ta=tc; td = tc; dqi = 0; dqf = 0;
            else
                argCheck = true; msg = ": tc must be less than (tf-ti)/2.";
            end
        else
            argCheck = true; msg = ": line 48 sqrt argument must be greater than zero.";
        end
    elseif ~isnan(dqi) && ~isnan(dqf) % dqi,dqf fixed
        if qf - qi < 0 
            % If dq < 0, invert the signs of initial vel/pos and after flip
            % the resulting trajectories
            qi = -qi; qf = -qf; sgn = -1; dqi = -dqi; dqf = -dqf;
        end
        if ~isnan(vm) && ~isnan(am) % ddqc, dqc max fixed
            if am*(qf-qi) > abs(dqi^2-dqf^2)*0.5 % Feasibility check
                ddqc = am;
                if am*(qf-qi) > vm^2-0.5*(dqi^2+dqf^2) % dqc max is reached 
                    dqc = vm;
                    ta = (vm-dqi)/am; td = (vm-dqf)/am;
                    % In this case tf is computed and not given
                    tf = (qf-qi)/vm + 0.5*vm*(1-dqi/vm)^2/am + 0.5*vm*(1-dqf/vm)^2/am + ti;
                else % dqc max is not reached
                    dqc = sqrt(am*(qf-qi)+0.5*(dqi^2+dqf^2));
                    % In this case tf is computed and not given
                    ta = (dqc-dqi)/am; td = (dqc-dqf)/am; tf = ta + td + ti;
                end
            else
                argCheck = true; msg = ": ddqc max is not feasible.";
            end
        elseif ~isnan(am) % ddqc max fixed
            if am*(qf-qi) > abs(dqi^2-dqf^2)*0.5 % Feasibility check
                f = am^2*(tf-ti)^2-4*am*(qf-qi)+2*am*(dqi+dqf)*(tf-ti)-(dqi-dqf)^2;
                if f >= 0 % Check sqrt argument
                    dqc = 0.5*(dqi+dqf+am*(tf-ti)-sqrt(f));
                    ddqc = am;
                    ta = (dqc-dqi)/am; td = (dqc-dqf)/am;
                else
                    argCheck = true; msg = ": line 84 sqrt argument must be greater than zero.";
                end
            else
                argCheck = true; msg = ": ddqc max is not feasible.";
            end
        end
    else % No constraint provided
        argCheck = true; msg = ": no constraint provided.";
    end
    % Trajectory generation
    if ~argCheck % Trajectory is feasible
        % Time intervals
        t1=ti:0.001:ta+ti;
        if size(t1) == 1 % Cover the case ta = 0
            t1 = [];
        end
        t2=ta+ti:0.001:tf-td;
        t3=tf-td:0.001:tf;
        if size(t3) == 1 % Cover the case td = 0
            t3 = [];
        end
        t = [t1 t2 t3];
        % Position
        q1 = @(t1) sgn*(qi + dqi*(t1-ti) + 0.5*(dqc-dqi)*(t1-ti).^2/ta);
        q2 = @(t2) sgn*(qi + 0.5*dqi*ta + dqc*(t2-ti-0.5*ta));
        q3 = @(t3) sgn*(qf - dqf*(tf-t3) - 0.5*(dqc-dqf)*(tf-t3).^2/td);
        q = [q1(t1) q2(t2) q3(t3)];
        % Velocity
        v1 = @(t1) sgn*(dqi + (dqc-dqi)*(t1-ti)/ta);
        v2 = @(t2) sgn*(dqc + 0*t2);
        v3 = @(t3) sgn*(dqf + (dqc-dqf)*(tf-t3)/td);
        v = [v1(t1) v2(t2) v3(t3)];
        % Acceleration
        a1 = @(t1) sgn*(ddqc + 0*t1);
        a2 = @(t2) 0*t2;
        a3 = @(t3) sgn*(-ddqc + 0*t3);
        a = [a1(t1) a2(t2) a3(t3)];
    else % Trajectory is not feasible
        disp("Trajectory not feasible" + msg);
        t = 0:0.001:1; 
        z = @(t) 0*t; q = z(t); v = z(t); a = z(t);
    end
end
%Params
AccelPerA = 3000;
phaseR = 0.033;
Ts = 0.001;
N = 50;
thetaFinal = 200;
x0 = [0;0];

%system definition
%X = [theta; omega]
Ac = [0 1;
     0 0];
Bc = [0;
     AccelPerA];
C = 0;
D = 0;

SYSC = ss(Ac, Bc, [], []);
SYSD = c2d(SYSC, Ts, 'zoh');
[Phi, Gamma] = predictionmatrices(SYSD.a, SYSD.b, SYSD.c, N);

Df = Gamma(end-1:end,:);
ff = [thetaFinal; 0];

H = 2*eye(N)*(3/2)*phaseR*Ts;
[u, fval] = quadprog(H,[],[],[],Df,ff);

xv = reshape(Gamma*u, 2,N)';
x = xv(:,1);
v = xv(:,2);

kv350_lambda = 2.2e-3;
power = u.*v.*(3/2)*kv350_lambda;

Vbus = 24;
Ib = power/Vbus;
Im = u;
duty = Ib./Im;
CapIsqr = (duty.*(Ib-Im)).^2 + ((1-duty).*Ib).^2;
CapIrms = sqrt(sum(CapIsqr)/N);
CapR = 0.08;
Ncaps = 8;
Cappow = (CapIrms/Ncaps)^2 * CapR

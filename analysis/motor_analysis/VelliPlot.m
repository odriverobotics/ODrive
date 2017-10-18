% limits
Imax = 64; %A
Umax = 22; %V
Irange = 100; %A. Range for plotting current
omegaMax = 2000; %rad/s mechanical. For plotting voltage ellipses
omegastep = 200; %rad/s mechanical. For plotting voltage ellipses


%%
%350 kv motor
lambda = 2.24/1000;
L = 23e-6;
R = 32e-3;
pp = 7; Poles = pp*2;
Ld = L;
Lq = L;

% %%
% % Donkey
% kv = 820;
% lambda = 60/(kv*2*pi*pp*sqrt(3));
% L = 8e-6; %Guess! TODO: measure
% R = 30e-3; %Guess! TODO: measure
% pp = 7; Poles = pp*2;
% Ld = L;
% Lq = L;

%%
Istep = Irange/400;
Idplt = repmat(-Irange:Istep:Irange,801,1);
Iqplt = repmat((-Irange:Istep:Irange)',1,801);
UmaxSq = (Umax/sqrt(3))^2;

t = linspace(0,2*pi);
IdMaxt = Imax*cos(t);
IqMaxt = Imax*sin(t);

%%
figure(1)
plot(IdMaxt, IqMaxt);

hold on;

%Plot torque
%[c,h] = contour(Idplt,Iqplt, (Poles/2).*(3/2).*(lambda.*Iqplt + (Ld-Lq).*Iqplt.*Idplt), -5:0.25:5);
%clabel(c,h,'LabelSpacing',500);

%Plot voltage ellipses
EllRHS = Ld^2.*(lambda/Ld + Idplt).^2 + Lq^2.*Iqplt.^2;
omegaAtEllipse = sqrt(UmaxSq./EllRHS);
[c,h] = contour(Idplt,Iqplt, omegaAtEllipse./(Poles/2), 0:omegastep:omegaMax);
clabel(c,h,'LabelSpacing',500);
xlabel 'Id (A)'
ylabel 'Iq (A)'
colormap(jet)
c = colorbar;
%c.Label.String = 'Speed (rad/s)';
ylabel(c,'Speed (mechanical rad/s)')
grid on
axis equal

%plot infinite speed point
plot(-lambda/Ld, 0, 'r*');

hold off;

%%
figure(2)

%fw range
t = linspace(pi/2,pi);
IdMaxt = Imax*cos(t);
IqMaxt = Imax*sin(t);
Tmaxt = (Poles/2).*(3/2).*(lambda.*IqMaxt + (Ld-Lq).*IqMaxt.*IdMaxt);
EllRHSmaxt = Ld^2.*(lambda/Ld + IdMaxt).^2 + Lq^2.*IqMaxt.^2;
omegamaxt = sqrt(UmaxSq./EllRHSmaxt);

%MTPA range
MTPAmaxtId = 0; %TODO make work for salient machines
MTPAmaxtIq = Imax;
TmaxtMTPA = (Poles/2).*(3/2).*(lambda.*MTPAmaxtIq + (Ld-Lq).*MTPAmaxtIq.*MTPAmaxtId);
Tmaxt = [repmat(TmaxtMTPA, 1, 100) Tmaxt];
omegamaxt = [linspace(0,omegamaxt(1)) omegamaxt];

%Present mechanical speed
omegamaxt = omegamaxt./(Poles/2);

Pmaxt = Tmaxt.*omegamaxt;

%plotyy(t, Tmaxt, t, omegamaxt);
%plotyy(t, Tmaxt, t, Pmaxt);
%plotyy(t, Pmaxt, t, omegamaxt);

h = plotyy(omegamaxt, Tmaxt, omegamaxt, Pmaxt);
grid on
xlabel 'Speed (mechanical rad/s)'
ylabel 'Torque (Nm)'
ylabel(h(2), 'Power (W)');
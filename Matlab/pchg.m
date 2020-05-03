function [pwr] = pchg(eirp, gr, wl, ht, r)
% returns: charging power in dBm
% eirp = EIRP of charger
% gr = receive antenna gain
% wl = wavelength in meters
% ht = hight in metres
% r = horizontal radius in metres
pwr = 10*log10(1000 * eirp * gr * (1-r^2/(ht^2 + r^2))^2 * (wl/(4*pi))^2 * 1/(ht^2 + r^2));

end

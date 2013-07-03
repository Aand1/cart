function mp_plot(tind,aind)
% mp_plot(tind,aind)
if nargin < 2
    disp('Error need at least two input arguments');
end
if tind < 0 | tind > 15
    disp('Error: first argument must be an integer between 0 and 15');
    return;
end
if aind < 0 | aind > 6
    disp('Error: second argument must be an integer between 0 and 6');
    return;
end

filename = strcat('actions/action_',int2str(tind),'_',int2str(aind));
A = load(filename);
filename = strcat('actions/action_rf_',int2str(tind),'_',int2str(aind));
B = load(filename);
filename = strcat('actions/action_sf_',int2str(tind),'_',int2str(aind));
C = load(filename);

%subplot(2,1,1)
hold on
plot(A(:,1),A(:,2),'*')
plot(C(:,1),C(:,2),'ro');
axis([-70 70 -70 70])
% subplot(2,1,2)
% hold on
% plot(B(:,1),B(:,2),'*')
% plot(C(:,1),C(:,2),'ro');
% axis([-70 70 -70 70])

